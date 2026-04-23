#include "can_driver/motor_maintenance_service.hpp"

#include "can_driver/AxisCommandSemantics.h"
#include "can_driver/EyouCan.h"
#include "can_driver/SafeCommand.h"

#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <urdf/model.h>

#include <algorithm>
#include <cmath>
#include <thread>

namespace {

struct ModeSelection {
    CanProtocol::MotorMode mode{CanProtocol::MotorMode::Position};
    can_driver::AxisControlMode controlMode{can_driver::AxisControlMode::Position};
};

bool decodeModeSelection(double value, ModeSelection *selection)
{
    if (selection == nullptr) {
        return false;
    }
    if (value == 0.0) {
        selection->controlMode = can_driver::AxisControlMode::Position;
        selection->mode = can_driver::protocolMotorModeFromAxisControlMode(selection->controlMode);
        return true;
    }
    if (value == 1.0) {
        selection->controlMode = can_driver::AxisControlMode::Velocity;
        selection->mode = can_driver::protocolMotorModeFromAxisControlMode(selection->controlMode);
        return true;
    }
    if (value == 2.0) {
        selection->controlMode = can_driver::AxisControlMode::Csp;
        selection->mode = can_driver::protocolMotorModeFromAxisControlMode(selection->controlMode);
        return true;
    }
    return false;
}

bool modeSwitchRequiresDisabledAxis(CanType protocol)
{
    // MT/DM 侧的 mode 切换主要用于驱动本地语义路由，不要求先失能。
    // PP/ECB 仍保留“先失能再改模式”的硬约束，避免在线切环引发突跳。
    return protocol == CanType::PP || protocol == CanType::ECB;
}

} // namespace

namespace {

bool writePositionOffset(const MotorMaintenanceService::JointConfig& target,
                         CanProtocol* protocol,
                         int32_t rawOffset,
                         bool persistToMotor,
                         std::string* message)
{
    if (protocol == nullptr) {
        if (message != nullptr) {
            *message = "Protocol not available.";
        }
        return false;
    }
    if (!protocol->setPositionOffset(target.motorId, rawOffset)) {
        if (message != nullptr) {
            *message = "Protocol rejected zero offset setting.";
        }
        return false;
    }
    if (!persistToMotor) {
        return true;
    }
    if (!protocol->persistParameters(target.motorId)) {
        if (message != nullptr) {
            *message = "Protocol rejected zero offset persistence.";
        }
        return false;
    }
    return true;
}

} // namespace

void MotorMaintenanceService::configure(
    ActivityChecker isActive,
    MotorActionExecutor *motorActionExecutor,
    JointLookup lookupJointByMotorId,
    DirectCommandClearer clearDirectCommand,
    ModeSwitchCommitter commitModeSwitch,
    ZeroCommitter commitZero,
    ZeroOffsetGetter getZeroOffset,
    LimitCommitter commitLimits,
    FreshFeedbackGetter getFreshAxisFeedback,
    DisabledRequirementChecker requireAxisDisabledForConfiguration,
    ProtocolGetter getProtocol,
    DeviceMutexGetter getDeviceMutex,
    bool persistZeroOffsetOnApplyToMotor)
{
    isActive_ = std::move(isActive);
    motorActionExecutor_ = motorActionExecutor;
    lookupJointByMotorId_ = std::move(lookupJointByMotorId);
    clearDirectCommand_ = std::move(clearDirectCommand);
    commitModeSwitch_ = std::move(commitModeSwitch);
    commitZero_ = std::move(commitZero);
    getZeroOffset_ = std::move(getZeroOffset);
    commitLimits_ = std::move(commitLimits);
    getFreshAxisFeedback_ = std::move(getFreshAxisFeedback);
    requireAxisDisabledForConfiguration_ = std::move(requireAxisDisabledForConfiguration);
    getProtocol_ = std::move(getProtocol);
    getDeviceMutex_ = std::move(getDeviceMutex);
    persistZeroOffsetOnApplyToMotor_ = persistZeroOffsetOnApplyToMotor;
}

void MotorMaintenanceService::initialize(ros::NodeHandle &pnh)
{
    initialize(pnh, AdvertiseOptions{});
}

void MotorMaintenanceService::initialize(ros::NodeHandle &pnh,
                                         const AdvertiseOptions &options)
{
    shutdown();
    if (options.motorCommand) {
        motorCmdSrv_ =
            pnh.advertiseService("motor_command", &MotorMaintenanceService::onMotorCommand, this);
    }
    if (options.setZero) {
        setZeroSrv_ =
            pnh.advertiseService("set_zero", &MotorMaintenanceService::onSetZero, this);
    }
    if (options.applyLimits) {
        applyLimitsSrv_ =
            pnh.advertiseService("apply_limits", &MotorMaintenanceService::onApplyLimits, this);
    }
    if (options.setZeroLimit) {
        setZeroLimitSrv_ =
            pnh.advertiseService("set_zero_limit", &MotorMaintenanceService::onSetZeroLimit, this);
    }
}

void MotorMaintenanceService::shutdown()
{
    motorCmdSrv_.shutdown();
    setZeroSrv_.shutdown();
    applyLimitsSrv_.shutdown();
    setZeroLimitSrv_.shutdown();
}

bool MotorMaintenanceService::lookupJointByMotorId(uint16_t motorId, JointConfig *joint) const
{
    if (!lookupJointByMotorId_ || joint == nullptr) {
        return false;
    }
    return lookupJointByMotorId_(motorId, joint);
}

MotorActionExecutor::Target
MotorMaintenanceService::makeMotorTarget(const JointConfig &joint) const
{
    return MotorActionExecutor::Target{joint.name, joint.canDevice, joint.protocol, joint.motorId};
}

bool MotorMaintenanceService::handleSetModeRequest(const JointConfig &target,
                                                   uint16_t motorId,
                                                   double modeValue,
                                                   std::string *message)
{
    if (!motorActionExecutor_ || !commitModeSwitch_) {
        if (message != nullptr) {
            *message = "Motor maintenance service not configured.";
        }
        return false;
    }

    ModeSelection selection;
    if (!decodeModeSelection(modeValue, &selection)) {
        if (message != nullptr) {
            *message = "CMD_SET_MODE value must be 0 (position), 1 (velocity) or 2 (csp).";
        }
        return false;
    }
    if (modeSwitchRequiresDisabledAxis(target.protocol)) {
        if (!requireAxisDisabledForConfiguration_ ||
            !requireAxisDisabledForConfiguration_(target, "Mode switch", message)) {
            if (message != nullptr && message->empty()) {
                *message = "Mode switch requires the motor to be disabled first.";
            }
            return false;
        }
    }

    const auto status = motorActionExecutor_->execute(
        makeMotorTarget(target),
        [&selection](const std::shared_ptr<CanProtocol> &proto, MotorID id) {
            return proto->setMode(id, selection.mode);
        },
        "Set mode");
    if (status != MotorActionExecutor::Status::Ok) {
        if (message != nullptr) {
            if (status == MotorActionExecutor::Status::DeviceNotReady) {
                *message = "CAN device not ready.";
            } else if (status == MotorActionExecutor::Status::ProtocolUnavailable) {
                *message = "Protocol not available.";
            } else if (status == MotorActionExecutor::Status::Rejected) {
                *message = "Set mode command rejected.";
            } else {
                *message = "Command execution failed.";
            }
        }
        return false;
    }
    if (!waitForPpModeConfirmation(target, selection.mode, message)) {
        return false;
    }

    JointConfig targetSnapshot;
    if (!lookupJointByMotorId(motorId, &targetSnapshot)) {
        if (message != nullptr) {
            *message = "Motor ID not found.";
        }
        return false;
    }

    int32_t preloadRaw = 0;
    bool shouldPreload = true;
    if (selection.mode != CanProtocol::MotorMode::Velocity) {
        if (!getFreshAxisFeedback_) {
            if (target.protocol == CanType::MT) {
                // MT 在无新鲜位置反馈时允许跳过 preload，由后续显式位置指令接管。
                shouldPreload = false;
            } else {
                if (message != nullptr) {
                    *message = "Current position feedback unavailable or stale for mode preload.";
                }
                return false;
            }
        } else {
            can_driver::SharedDriverState::AxisFeedbackState feedback;
            if (!getFreshAxisFeedback_(targetSnapshot, &feedback) || !feedback.positionValid) {
                if (target.protocol == CanType::MT) {
                    // MT 场景允许无新鲜反馈切环，避免“能控但切不了模式”。
                    shouldPreload = false;
                } else {
                    if (message != nullptr) {
                        *message = "Current position feedback unavailable or stale for mode preload.";
                    }
                    return false;
                }
            } else {
                preloadRaw = can_driver::safe_command::clampToInt32(feedback.position);
            }
        }
    }

    if (shouldPreload) {
        const auto preloadStatus = motorActionExecutor_->execute(
            makeMotorTarget(targetSnapshot),
            [selection, preloadRaw](const std::shared_ptr<CanProtocol> &proto, MotorID id) {
                switch (selection.mode) {
                case CanProtocol::MotorMode::Position:
                    return proto->setPosition(id, preloadRaw);
                case CanProtocol::MotorMode::Velocity:
                    return proto->setVelocity(id, 0);
                case CanProtocol::MotorMode::CSP:
                    return proto->quickSetPosition(id, preloadRaw);
                }
                return false;
            },
            "Preload mode command");
        if (preloadStatus != MotorActionExecutor::Status::Ok) {
            if (message != nullptr) {
                if (preloadStatus == MotorActionExecutor::Status::DeviceNotReady) {
                    *message = "CAN device not ready.";
                } else if (preloadStatus == MotorActionExecutor::Status::ProtocolUnavailable) {
                    *message = "Protocol not available.";
                } else if (preloadStatus == MotorActionExecutor::Status::Rejected) {
                    *message = "Set mode preload command rejected.";
                } else {
                    *message = "Command execution failed.";
                }
            } else {
                return false;
            }
            return false;
        }
    }

    if (!commitModeSwitch_(motorId, selection.controlMode)) {
        if (message != nullptr) {
            *message = "Failed to update local joint mode state.";
        }
        return false;
    }
    if (message != nullptr) {
        *message = "OK";
    }
    return true;
}

bool MotorMaintenanceService::SetModeByMotorId(uint16_t motorId,
                                               double modeValue,
                                               std::string *message)
{
    if (message != nullptr) {
        message->clear();
    }
    if (!isActive_ || !isActive_()) {
        if (message != nullptr) {
            *message = "Driver inactive.";
        }
        return false;
    }

    JointConfig target;
    if (!lookupJointByMotorId(motorId, &target)) {
        if (message != nullptr) {
            *message = "Motor ID not found.";
        }
        return false;
    }

    return handleSetModeRequest(target, motorId, modeValue, message);
}

bool MotorMaintenanceService::resolveBaseLimits(const JointConfig& target,
                                                bool useUrdfLimits,
                                                double requestedMinRad,
                                                double requestedMaxRad,
                                                ResolvedLimits* out,
                                                std::string* message) const
{
    if (out == nullptr) {
        if (message != nullptr) {
            *message = "Resolved limit output is null.";
        }
        return false;
    }

    out->baseMin = requestedMinRad;
    out->baseMax = requestedMaxRad;
    if (!useUrdfLimits) {
        return true;
    }

    urdf::Model urdf;
    if (!urdf.initParam("robot_description")) {
        if (message != nullptr) {
            *message = "robot_description not found; cannot read URDF limits.";
        }
        return false;
    }
    auto urdfJoint = urdf.getJoint(target.name);
    if (!urdfJoint) {
        if (message != nullptr) {
            *message = "Joint not found in URDF: " + target.name;
        }
        return false;
    }
    joint_limits_interface::JointLimits urdfLimits;
    if (!joint_limits_interface::getJointLimits(urdfJoint, urdfLimits) ||
        !urdfLimits.has_position_limits) {
        if (message != nullptr) {
            *message = "URDF has no position limits for joint: " + target.name;
        }
        return false;
    }
    out->baseMin = urdfLimits.min_position;
    out->baseMax = urdfLimits.max_position;
    return true;
}

bool MotorMaintenanceService::resolveCurrentReportedPosition(
    const JointConfig& target,
    double* currentPositionRad,
    bool* hasFreshFeedback,
    std::string* message) const
{
    if (currentPositionRad == nullptr || hasFreshFeedback == nullptr) {
        if (message != nullptr) {
            *message = "Current position output is null.";
        }
        return false;
    }

    *currentPositionRad = 0.0;
    *hasFreshFeedback = false;

    if (!getProtocol_ || !getDeviceMutex_) {
        if (message != nullptr) {
            *message = "Protocol not available.";
        }
        return false;
    }
    auto proto = getProtocol_(target.canDevice, target.protocol);
    auto devMutex = getDeviceMutex_(target.canDevice);
    if (!proto || !devMutex) {
        if (message != nullptr) {
            *message = "Protocol not available.";
        }
        return false;
    }

    int64_t rawPos = 0;
    can_driver::SharedDriverState::AxisFeedbackState feedback;
    if (getFreshAxisFeedback_ && getFreshAxisFeedback_(target, &feedback) &&
        feedback.positionValid) {
        rawPos = feedback.position;
        *hasFreshFeedback = true;
    } else {
        std::lock_guard<std::mutex> devLock(*devMutex);
        for (int i = 0; i < 5; ++i) {
            rawPos = proto->getPosition(target.motorId);
            std::this_thread::sleep_for(std::chrono::milliseconds(15));
        }
    }

    *currentPositionRad =
        static_cast<double>(rawPos) * can_driver::effectivePositionScale(target);
    return true;
}

bool MotorMaintenanceService::resolveCurrentZeroOffset(const JointConfig& target,
                                                       bool applyToMotor,
                                                       double* zeroOffsetRad,
                                                       std::string* message) const
{
    if (zeroOffsetRad == nullptr) {
        if (message != nullptr) {
            *message = "Zero offset output is null.";
        }
        return false;
    }
    *zeroOffsetRad = 0.0;

    if (applyToMotor && getProtocol_) {
        auto proto = getProtocol_(target.canDevice, target.protocol);
        int32_t offsetRaw = 0;
        if (proto && proto->readPositionOffset(target.motorId, &offsetRaw)) {
            *zeroOffsetRad =
                static_cast<double>(offsetRaw) * can_driver::effectivePositionScale(target);
            return true;
        }
    }

    if (getZeroOffset_ && getZeroOffset_(static_cast<uint16_t>(target.motorId), zeroOffsetRad)) {
        return true;
    }

    *zeroOffsetRad = 0.0;
    return true;
}

bool MotorMaintenanceService::SetZeroByMotorId(uint16_t motorId,
                                               double zeroOffsetRad,
                                               bool useCurrentPositionAsZero,
                                               bool applyToMotor,
                                               double* previousZeroOffsetRad,
                                               double* currentPositionRad,
                                               double* appliedZeroOffsetRad,
                                               std::string* message)
{
    if (message != nullptr) {
        message->clear();
    }
    if (previousZeroOffsetRad != nullptr) {
        *previousZeroOffsetRad = 0.0;
    }
    if (currentPositionRad != nullptr) {
        *currentPositionRad = 0.0;
    }
    if (appliedZeroOffsetRad != nullptr) {
        *appliedZeroOffsetRad = 0.0;
    }

    if (!isActive_ || !isActive_()) {
        if (message != nullptr) {
            *message = "Driver inactive.";
        }
        return false;
    }

    JointConfig target;
    if (!lookupJointByMotorId(motorId, &target)) {
        if (message != nullptr) {
            *message = "Motor ID not found.";
        }
        return false;
    }
    if (!can_driver::controlModeUsesPositionSemantics(target.controlMode)) {
        if (message != nullptr) {
            *message = "Only position-semantic joints support zero/position limits.";
        }
        return false;
    }
    if (useCurrentPositionAsZero && !applyToMotor) {
        if (message != nullptr) {
            *message = "Automatic zeroing requires apply_to_motor=true.";
        }
        return false;
    }

    double currentReportedPositionRad = 0.0;
    bool hasFreshFeedback = false;
    if (!resolveCurrentReportedPosition(target,
                                        &currentReportedPositionRad,
                                        &hasFreshFeedback,
                                        message)) {
        return false;
    }
    if (currentPositionRad != nullptr) {
        *currentPositionRad = currentReportedPositionRad;
    }
    if (useCurrentPositionAsZero && !hasFreshFeedback) {
        if (message != nullptr) {
            *message = "Current position feedback unavailable or stale; cannot auto-zero.";
        }
        return false;
    }

    double previousZeroOffset = 0.0;
    if (!resolveCurrentZeroOffset(target, applyToMotor, &previousZeroOffset, message)) {
        return false;
    }
    if (previousZeroOffsetRad != nullptr) {
        *previousZeroOffsetRad = previousZeroOffset;
    }

    const double currentPhysicalPositionRad =
        currentReportedPositionRad - previousZeroOffset;
    const double resolvedZeroOffset =
        useCurrentPositionAsZero ? -currentPhysicalPositionRad : zeroOffsetRad;
    if (appliedZeroOffsetRad != nullptr) {
        *appliedZeroOffsetRad = resolvedZeroOffset;
    }

    if (applyToMotor) {
        if (!requireAxisDisabledForConfiguration_ ||
            !requireAxisDisabledForConfiguration_(
                target, "Writing zero offset", message)) {
            if (message != nullptr && message->empty()) {
                *message = "Writing zero offset requires the motor to be disabled first.";
            }
            return false;
        }

        int32_t rawOffset = 0;
        if (!can_driver::safe_command::scaleAndClampToInt32(
                resolvedZeroOffset,
                can_driver::effectivePositionScale(target),
                target.name + ".zero_offset",
                rawOffset)) {
            if (message != nullptr) {
                *message = "Failed to convert zero offset to protocol raw value.";
            }
            return false;
        }

        int32_t previousRawOffset = 0;
        if (!can_driver::safe_command::scaleAndClampToInt32(
                previousZeroOffset,
                can_driver::effectivePositionScale(target),
                target.name + ".previous_zero_offset",
                previousRawOffset)) {
            if (message != nullptr) {
                *message = "Failed to convert previous zero offset to protocol raw value.";
            }
            return false;
        }

        if (!motorActionExecutor_) {
            if (message != nullptr) {
                *message = "Motor maintenance service not configured.";
            }
            return false;
        }

        const bool persistToMotor = persistZeroOffsetOnApplyToMotor_;
        bool persistOk = false;
        std::string persistError = "Set zero execution failed.";
        const auto status = motorActionExecutor_->execute(
            makeMotorTarget(target),
            [&target, rawOffset, previousRawOffset, persistToMotor, &persistOk, &persistError](
                const std::shared_ptr<CanProtocol>& p, MotorID /*id*/) {
                if (!writePositionOffset(
                        target, p.get(), rawOffset, persistToMotor, &persistError)) {
                    std::string rollbackError;
                    if (!writePositionOffset(
                            target, p.get(), previousRawOffset, persistToMotor, &rollbackError)) {
                        persistError += " Rollback failed: " + rollbackError;
                    } else {
                        persistError += " Rolled back previous zero offset.";
                    }
                    persistOk = false;
                    return false;
                }
                persistOk = true;
                return true;
            },
            "Set zero offset");
        if (status != MotorActionExecutor::Status::Ok) {
            if (message != nullptr) {
                if (status == MotorActionExecutor::Status::DeviceNotReady) {
                    *message = "CAN device not ready.";
                } else if (status == MotorActionExecutor::Status::ProtocolUnavailable) {
                    *message = "Protocol not available.";
                } else if (status == MotorActionExecutor::Status::Rejected) {
                    *message = persistError;
                } else {
                    *message = "Set zero execution failed.";
                }
            }
            return false;
        }
        if (!persistOk) {
            if (message != nullptr) {
                *message = persistError;
            }
            return false;
        }
    }

    if (!commitZero_ || !commitZero_(motorId, resolvedZeroOffset, previousZeroOffset)) {
        if (message != nullptr) {
            *message = "Failed to update local zero state.";
        }
        return false;
    }
    if (message != nullptr) {
        *message = "OK";
    }
    return true;
}

bool MotorMaintenanceService::ApplyLimitsByMotorId(uint16_t motorId,
                                                   double minPositionRad,
                                                   double maxPositionRad,
                                                   bool useUrdfLimits,
                                                   bool applyToMotor,
                                                   bool requireCurrentInsideLimits,
                                                   double* currentPositionRad,
                                                   double* activeZeroOffsetRad,
                                                   double* appliedMinRad,
                                                   double* appliedMaxRad,
                                                   std::string* message)
{
    if (message != nullptr) {
        message->clear();
    }
    if (currentPositionRad != nullptr) {
        *currentPositionRad = 0.0;
    }
    if (activeZeroOffsetRad != nullptr) {
        *activeZeroOffsetRad = 0.0;
    }
    if (appliedMinRad != nullptr) {
        *appliedMinRad = 0.0;
    }
    if (appliedMaxRad != nullptr) {
        *appliedMaxRad = 0.0;
    }

    if (!isActive_ || !isActive_()) {
        if (message != nullptr) {
            *message = "Driver inactive.";
        }
        return false;
    }

    JointConfig target;
    if (!lookupJointByMotorId(motorId, &target)) {
        if (message != nullptr) {
            *message = "Motor ID not found.";
        }
        return false;
    }
    if (!can_driver::controlModeUsesPositionSemantics(target.controlMode)) {
        if (message != nullptr) {
            *message = "Only position-semantic joints support zero/position limits.";
        }
        return false;
    }

    ResolvedLimits limits;
    if (!resolveBaseLimits(target,
                           useUrdfLimits,
                           minPositionRad,
                           maxPositionRad,
                           &limits,
                           message)) {
        return false;
    }

    double currentReportedPositionRad = 0.0;
    bool hasFreshFeedback = false;
    if (!resolveCurrentReportedPosition(target,
                                        &currentReportedPositionRad,
                                        &hasFreshFeedback,
                                        message)) {
        return false;
    }
    (void)hasFreshFeedback;
    if (currentPositionRad != nullptr) {
        *currentPositionRad = currentReportedPositionRad;
    }

    double activeZeroOffset = 0.0;
    if (!resolveCurrentZeroOffset(target, applyToMotor, &activeZeroOffset, message)) {
        return false;
    }
    if (activeZeroOffsetRad != nullptr) {
        *activeZeroOffsetRad = activeZeroOffset;
    }

    const double resolvedMin = limits.baseMin;
    const double resolvedMax = limits.baseMax;
    if (!std::isfinite(resolvedMin) || !std::isfinite(resolvedMax) ||
        resolvedMin >= resolvedMax) {
        if (message != nullptr) {
            *message = "Invalid limit range.";
        }
        return false;
    }
    if (appliedMinRad != nullptr) {
        *appliedMinRad = resolvedMin;
    }
    if (appliedMaxRad != nullptr) {
        *appliedMaxRad = resolvedMax;
    }

    if (requireCurrentInsideLimits &&
        (currentReportedPositionRad < resolvedMin ||
         currentReportedPositionRad > resolvedMax)) {
        if (message != nullptr) {
            *message = "Current position is outside requested limit range.";
        }
        return false;
    }

    if (applyToMotor) {
        if (!requireAxisDisabledForConfiguration_ ||
            !requireAxisDisabledForConfiguration_(
                target, "Writing hardware limits", message)) {
            if (message != nullptr && message->empty()) {
                *message = "Writing hardware limits requires the motor to be disabled first.";
            }
            return false;
        }

        int32_t rawMin = 0;
        int32_t rawMax = 0;
        int32_t rawLimitA = 0;
        int32_t rawLimitB = 0;
        if (!can_driver::safe_command::scaleAndClampToInt32(
                resolvedMin,
                can_driver::effectivePositionScale(target),
                target.name + ".min_limit",
                rawLimitA) ||
            !can_driver::safe_command::scaleAndClampToInt32(
                resolvedMax,
                can_driver::effectivePositionScale(target),
                target.name + ".max_limit",
                rawLimitB)) {
            if (message != nullptr) {
                *message = "Failed to convert limits to protocol raw values.";
            }
            return false;
        }
        const auto orderedRawLimits = std::minmax(rawLimitA, rawLimitB);
        rawMin = orderedRawLimits.first;
        rawMax = orderedRawLimits.second;

        if (!motorActionExecutor_) {
            if (message != nullptr) {
                *message = "Motor maintenance service not configured.";
            }
            return false;
        }
        const auto status = motorActionExecutor_->execute(
            makeMotorTarget(target),
            [rawMin, rawMax](const std::shared_ptr<CanProtocol>& p, MotorID id) {
                return p->configurePositionLimits(id, rawMin, rawMax, true);
            },
            "Set position limits");
        if (status != MotorActionExecutor::Status::Ok) {
            if (message != nullptr) {
                if (status == MotorActionExecutor::Status::DeviceNotReady) {
                    *message = "CAN device not ready.";
                } else if (status == MotorActionExecutor::Status::ProtocolUnavailable) {
                    *message = "Protocol not available.";
                } else if (status == MotorActionExecutor::Status::Rejected) {
                    *message = "Protocol rejected position limit setting.";
                } else {
                    *message = "Set limits execution failed.";
                }
            }
            return false;
        }
    }

    if (!commitLimits_ ||
        !commitLimits_(motorId, resolvedMin, resolvedMax, activeZeroOffset)) {
        if (message != nullptr) {
            *message = "Failed to update local limit state.";
        }
        return false;
    }

    if (message != nullptr) {
        *message = "OK";
    }
    return true;
}

bool MotorMaintenanceService::waitForPpModeConfirmation(const JointConfig &joint,
                                                        CanProtocol::MotorMode expectedMode,
                                                        std::string *message) const
{
    if (joint.protocol != CanType::PP || !getProtocol_) {
        return true;
    }

    auto protocol = std::dynamic_pointer_cast<EyouCan>(getProtocol_(joint.canDevice, joint.protocol));
    if (!protocol) {
        return true;
    }
    if (!getFreshAxisFeedback_) {
        if (message != nullptr) {
            *message = "Mode confirmation requires fresh feedback access.";
        }
        return false;
    }

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
    while (std::chrono::steady_clock::now() < deadline) {
        protocol->issueRefreshQuery(joint.motorId, EyouCan::RefreshQuery::Mode);

        can_driver::SharedDriverState::AxisFeedbackState feedback;
        if (getFreshAxisFeedback_(joint, &feedback) && feedback.modeValid &&
            feedback.mode == expectedMode) {
            return true;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    if (message != nullptr) {
        *message = "Timed out waiting for mode confirmation from PP feedback.";
    }
    return false;
}

bool MotorMaintenanceService::onMotorCommand(can_driver::MotorCommand::Request &req,
                                             can_driver::MotorCommand::Response &res)
{
    if (!isActive_ || !isActive_()) {
        res.success = false;
        res.message = "Driver inactive.";
        return true;
    }

    JointConfig target;
    if (!lookupJointByMotorId(req.motor_id, &target)) {
        res.success = false;
        res.message = "Motor ID not found.";
        return true;
    }

    auto handleFailure = [&res](MotorActionExecutor::Status status, const char *rejectedMsg) {
        if (status == MotorActionExecutor::Status::DeviceNotReady) {
            res.success = false;
            res.message = "CAN device not ready.";
        } else if (status == MotorActionExecutor::Status::ProtocolUnavailable) {
            res.success = false;
            res.message = "Protocol not available.";
        } else if (status == MotorActionExecutor::Status::Rejected) {
            res.success = false;
            res.message = rejectedMsg;
        } else {
            res.success = false;
            res.message = "Command execution failed.";
        }
    };

    if (req.command == can_driver::MotorCommand::Request::CMD_SET_MODE) {
        res.success = handleSetModeRequest(target, req.motor_id, req.value, &res.message);
        return true;
    }

    struct CmdEntry {
        uint8_t cmd;
        const char *name;
        bool clearDirect;
        std::function<bool(const std::shared_ptr<CanProtocol> &, MotorID)> action;
    };
    const std::vector<CmdEntry> table = {
        {can_driver::MotorCommand::Request::CMD_ENABLE,
         "Enable",
         false,
         [](const std::shared_ptr<CanProtocol> &proto, MotorID id) { return proto->Enable(id); }},
        {can_driver::MotorCommand::Request::CMD_DISABLE,
         "Disable",
         true,
         [](const std::shared_ptr<CanProtocol> &proto, MotorID id) { return proto->Disable(id); }},
        {can_driver::MotorCommand::Request::CMD_STOP,
         "Stop",
         true,
         [](const std::shared_ptr<CanProtocol> &proto, MotorID id) { return proto->Stop(id); }},
    };

    for (const auto &entry : table) {
        if (req.command != entry.cmd) {
            continue;
        }
        if (!motorActionExecutor_) {
            res.success = false;
            res.message = "Motor maintenance service not configured.";
            return true;
        }
        const auto status =
            motorActionExecutor_->execute(makeMotorTarget(target), entry.action, entry.name);
        if (status != MotorActionExecutor::Status::Ok) {
            const std::string rejectedMsg = std::string(entry.name) + " command rejected.";
            handleFailure(status, rejectedMsg.c_str());
            return true;
        }
        if (entry.clearDirect && clearDirectCommand_) {
            clearDirectCommand_(target.name);
        }
        res.success = true;
        res.message = "OK";
        return true;
    }

    res.success = false;
    res.message = "Unknown command.";
    return true;
}

bool MotorMaintenanceService::onSetZero(can_driver::SetZero::Request &req,
                                        can_driver::SetZero::Response &res)
{
    res.success = SetZeroByMotorId(req.motor_id,
                                   req.zero_offset_rad,
                                   req.use_current_position_as_zero,
                                   req.apply_to_motor,
                                   &res.previous_zero_offset_rad,
                                   &res.current_position_rad,
                                   &res.applied_zero_offset_rad,
                                   &res.message);
    return true;
}

bool MotorMaintenanceService::onApplyLimits(can_driver::ApplyLimits::Request &req,
                                            can_driver::ApplyLimits::Response &res)
{
    res.success = ApplyLimitsByMotorId(req.motor_id,
                                       req.min_position_rad,
                                       req.max_position_rad,
                                       req.use_urdf_limits,
                                       req.apply_to_motor,
                                       req.require_current_inside_limits,
                                       &res.current_position_rad,
                                       &res.active_zero_offset_rad,
                                       &res.applied_min_rad,
                                       &res.applied_max_rad,
                                       &res.message);
    return true;
}

bool MotorMaintenanceService::onSetZeroLimit(can_driver::SetZeroLimit::Request &req,
                                             can_driver::SetZeroLimit::Response &res)
{
    double previousZeroOffset = 0.0;
    res.success = SetZeroByMotorId(req.motor_id,
                                   req.zero_offset_rad,
                                   req.use_current_position_as_zero,
                                   req.apply_to_motor,
                                   &previousZeroOffset,
                                   &res.current_position_rad,
                                   &res.applied_zero_offset_rad,
                                   &res.message);
    if (!res.success) {
        return true;
    }

    res.success = ApplyLimitsByMotorId(req.motor_id,
                                       req.min_position_rad,
                                       req.max_position_rad,
                                       req.use_urdf_limits,
                                       req.apply_to_motor,
                                       true,
                                       &res.current_position_rad,
                                       &res.applied_zero_offset_rad,
                                       &res.applied_min_rad,
                                       &res.applied_max_rad,
                                       &res.message);
    return true;
}
