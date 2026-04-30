#ifndef CAN_DRIVER_CAN_DRIVER_IO_RUNTIME_H
#define CAN_DRIVER_CAN_DRIVER_IO_RUNTIME_H

#include "can_driver/AxisCommandSemantics.h"
#include "can_driver/AxisReadinessEvaluator.h"
#include "can_driver/CanDriverHwTypes.h"
#include "can_driver/IDeviceManager.h"
#include "can_driver/SafeCommand.h"
#include "can_driver/SharedDriverState.h"
#include "can_driver/command_gate.hpp"

#include <can_driver/MotorState.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <exception>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <ros/ros.h>

namespace can_driver {

class CanDriverIoRuntime {
public:
    struct WriteConfig {
        double directCmdTimeoutSec{0.5};
        bool debugBypassRosControl{false};
        bool safetyStopOnFault{true};
        bool safetyRequireEnabledForMotion{true};
        double maxPositionStepRad{0.0};
        bool safetyHoldAfterDeviceRecover{true};
        std::int64_t feedbackFreshnessTimeoutNs{AxisReadinessEvaluator::Config{}.feedbackFreshnessTimeoutNs};
    };

    struct PublishResult {
        std::vector<can_driver::MotorState> messages;
        bool anyFault{false};
    };

    static void SyncJointFeedback(const IDeviceManager &deviceManager,
                                  const std::vector<CanDriverDeviceProtocolGroup> &groups,
                                  std::deque<CanDriverJointConfig> *joints,
                                  std::mutex *jointStateMutex)
    {
        if (joints == nullptr || jointStateMutex == nullptr) {
            return;
        }

        struct JointSnapshot {
            double pos{0.0};
            double vel{0.0};
            double eff{0.0};
            bool valid{false};
        };

        std::vector<JointSnapshot> snapshots(joints->size());
        for (const auto &group : groups) {
            auto proto = deviceManager.getProtocol(group.canDevice, group.protocol);
            auto devMutex = deviceManager.getDeviceMutex(group.canDevice);
            if (!proto || !devMutex) {
                continue;
            }

            std::lock_guard<std::mutex> devLock(*devMutex);
            for (const std::size_t index : group.jointIndices) {
                const auto &joint = (*joints)[index];
                snapshots[index].pos =
                    can_driver::rawPositionToJointPosition(
                        joint,
                        static_cast<double>(proto->getPosition(joint.motorId)));
                snapshots[index].vel =
                    static_cast<double>(proto->getVelocity(joint.motorId)) *
                    can_driver::effectiveVelocityScale(joint);
                snapshots[index].eff = static_cast<double>(proto->getCurrent(joint.motorId));
                snapshots[index].valid = true;
            }
        }

        std::lock_guard<std::mutex> lock(*jointStateMutex);
        for (std::size_t index = 0; index < joints->size(); ++index) {
            if (!snapshots[index].valid) {
                continue;
            }
            (*joints)[index].pos = snapshots[index].pos;
            (*joints)[index].vel = snapshots[index].vel;
            (*joints)[index].eff = snapshots[index].eff;
        }
    }

    static void PrepareCommands(std::deque<CanDriverJointConfig> *joints,
                                std::vector<int32_t> *rawCommandBuffer,
                                std::vector<uint8_t> *commandValidBuffer,
                                std::vector<CanDriverPreparedCommand> *preparedCommandBuffer,
                                std::mutex *jointStateMutex,
                                const WriteConfig &config)
    {
        if (joints == nullptr || rawCommandBuffer == nullptr || commandValidBuffer == nullptr ||
            preparedCommandBuffer == nullptr || jointStateMutex == nullptr) {
            return;
        }

        std::lock_guard<std::mutex> lock(*jointStateMutex);
        const ros::Time now = ros::Time::now();
        bool activeDirectCommandPresent = false;
        if (!config.debugBypassRosControl) {
            for (auto &joint : *joints) {
                const auto mode = can_driver::axisControlModeFromString(joint.controlMode);
                if (!can_driver::controlModeHasDirectCommand(joint, mode)) {
                    continue;
                }
                const double age =
                    (now - can_driver::controlModeLastDirectCommandTime(joint, mode)).toSec();
                if (age <= config.directCmdTimeoutSec) {
                    activeDirectCommandPresent = true;
                    break;
                }
                can_driver::clearDirectCommandForControlMode(&joint, mode);
            }
        }
        for (std::size_t index = 0; index < joints->size(); ++index) {
            auto &joint = (*joints)[index];
            const auto mode = can_driver::axisControlModeFromString(joint.controlMode);
            auto &prepared = (*preparedCommandBuffer)[index];
            prepared.valid = false;
            prepared.jointIndex = index;
            prepared.motorId = joint.motorId;
            prepared.route = can_driver::controlModeDispatchRoute(mode);
            prepared.jointName = joint.name;

            bool useDirect = false;
            double cmdValue = 0.0;
            bool hasCommand = true;
            if (can_driver::controlModeHasDirectCommand(joint, mode)) {
                if (config.debugBypassRosControl) {
                    useDirect = true;
                    cmdValue = can_driver::controlModeDirectCommandValue(joint, mode);
                } else {
                    const double age =
                        (now - can_driver::controlModeLastDirectCommandTime(joint, mode)).toSec();
                    if (age <= config.directCmdTimeoutSec) {
                        useDirect = true;
                        cmdValue = can_driver::controlModeDirectCommandValue(joint, mode);
                    } else {
                        can_driver::clearDirectCommandForControlMode(&joint, mode);
                    }
                }
            }
            if (!useDirect) {
                if (config.debugBypassRosControl || activeDirectCommandPresent) {
                    hasCommand = false;
                } else {
                    cmdValue = can_driver::controlModeFallbackCommandValue(joint, mode);
                }
            }

            if (!hasCommand) {
                (*commandValidBuffer)[index] = 0;
                continue;
            }

            cmdValue = clampWithJointLimits(joint, cmdValue);
            if (joint.requireCommandAlignment) {
                const double actualValue = can_driver::controlModeActualFeedbackValue(joint, mode);
                const double tolerance = can_driver::controlModeAlignmentTolerance(joint, mode);
                if (!std::isfinite(cmdValue) || !std::isfinite(actualValue) ||
                    std::fabs(cmdValue - actualValue) > tolerance) {
                    (*commandValidBuffer)[index] = 0;
                    ROS_WARN_THROTTLE(
                        1.0,
                        "[CanDriverHW] Joint '%s' waiting for an aligned %s command after mode switch: actual=%.6f target=%.6f tolerance=%.6f.",
                        joint.name.c_str(),
                        can_driver::controlModeSemanticLabel(mode),
                        actualValue,
                        cmdValue,
                        tolerance);
                    continue;
                }
                joint.requireCommandAlignment = false;
            }
            if (can_driver::controlModeUsesPositionSemantics(joint.controlMode) &&
                config.maxPositionStepRad > 0.0 &&
                std::isfinite(cmdValue) && std::isfinite(joint.pos)) {
                const double delta = cmdValue - joint.pos;
                if (std::fabs(delta) > config.maxPositionStepRad) {
                    const double limitedCmd =
                        joint.pos + std::copysign(config.maxPositionStepRad, delta);
                    ROS_WARN_THROTTLE(
                        1.0,
                        "[CanDriverHW] Joint '%s' position step limited: cur=%.6f, req=%.6f, limited=%.6f (max_step=%.6f).",
                        joint.name.c_str(),
                        joint.pos,
                        cmdValue,
                        limitedCmd,
                        config.maxPositionStepRad);
                    cmdValue = limitedCmd;
                }
                cmdValue = clampWithJointLimits(joint, cmdValue);
            }

            double rawCmdValue = cmdValue;
            if (can_driver::controlModeUsesPositionSemantics(mode)) {
                rawCmdValue = can_driver::jointPositionToRawPositionCommand(joint, cmdValue);
            }
            const double scale = can_driver::controlModeScale(joint, mode);
            (*commandValidBuffer)[index] = static_cast<uint8_t>(
                can_driver::safe_command::scaleAndClampToInt32(
                    rawCmdValue, scale, joint.name, (*rawCommandBuffer)[index]));
            prepared.valid = ((*commandValidBuffer)[index] != 0);
        }
    }

    static void DispatchPreparedCommands(const IDeviceManager &deviceManager,
                                         const std::vector<CanDriverDeviceProtocolGroup> &groups,
                                         std::deque<CanDriverJointConfig> *joints,
                                         const std::vector<int32_t> &rawCommandBuffer,
                                         std::vector<uint8_t> *commandValidBuffer,
                                         const std::vector<CanDriverPreparedCommand> &preparedCommandBuffer,
                                         std::mutex *jointStateMutex,
                                         CommandGate *commandGate,
                                         const WriteConfig &config,
                                         bool *anyFaultObserved)
    {
        if (joints == nullptr || commandValidBuffer == nullptr || jointStateMutex == nullptr ||
            commandGate == nullptr) {
            return;
        }

        struct MotionStatusSnapshot {
            bool enabled{false};
            bool fault{false};
            bool valid{false};
        };

        const auto sharedState = deviceManager.getSharedDriverState();

        for (const auto &group : groups) {
            const bool isReady = deviceManager.isDeviceReady(group.canDevice);
            const auto deviceEvent = commandGate->observeDeviceReady(group.canDevice, isReady);

            if (!isReady) {
                if (deviceEvent == CommandGate::DeviceEvent::Lost) {
                    ROS_ERROR_THROTTLE(
                        1.0,
                        "[CanDriverHW] Device '%s' not ready, clear direct commands and block motion.",
                        group.canDevice.c_str());
                } else {
                    ROS_WARN_THROTTLE(
                        1.0,
                        "[CanDriverHW] Device '%s' not ready, skip command write.",
                        group.canDevice.c_str());
                }

                clearGroupCommands(group, joints, commandValidBuffer, jointStateMutex);
                continue;
            }

            if (config.safetyHoldAfterDeviceRecover &&
                deviceEvent == CommandGate::DeviceEvent::Recovered) {
                ROS_WARN_THROTTLE(
                    1.0,
                    "[CanDriverHW] Device '%s' recovered, hold one cycle and require fresh command.",
                    group.canDevice.c_str());
                commandGate->holdCommands();
                commandGate->armFreshCommandLatch();
                continue;
            }

            std::vector<MotionStatusSnapshot> motionStatusSnapshots(joints->size());
            if (sharedState) {
                const auto nowNs = SharedDriverSteadyNowNs();
                for (const std::size_t index : group.jointIndices) {
                    SharedDriverState::AxisFeedbackState feedback;
                    if (!sharedState->getAxisFeedback(
                            MakeAxisKey(group.canDevice,
                                        group.protocol,
                                        preparedCommandBuffer[index].motorId),
                            &feedback) ||
                        !sharedFeedbackFresh(feedback, nowNs, config.feedbackFreshnessTimeoutNs)) {
                        continue;
                    }

                    motionStatusSnapshots[index].enabled = feedback.enabled;
                    motionStatusSnapshots[index].fault = feedback.fault;
                    motionStatusSnapshots[index].valid = true;
                }
            }

            auto proto = deviceManager.getProtocol(group.canDevice, group.protocol);
            auto devMutex = deviceManager.getDeviceMutex(group.canDevice);
            if (!proto || !devMutex) {
                continue;
            }

            std::lock_guard<std::mutex> devLock(*devMutex);

            for (const std::size_t index : group.jointIndices) {
                const auto &prepared = preparedCommandBuffer[index];
                if (!prepared.valid || !(*commandValidBuffer)[index]) {
                    continue;
                }

                try {
                    const bool hasSharedMotionStatus = motionStatusSnapshots[index].valid;
                    const bool hasFault = hasSharedMotionStatus
                                              ? motionStatusSnapshots[index].fault
                                              : proto->hasFault(prepared.motorId);

                    if (config.safetyStopOnFault) {
                        bool needIssueStop = false;
                        {
                            std::lock_guard<std::mutex> lock(*jointStateMutex);
                            if (hasFault) {
                                if (!(*joints)[index].stopIssuedOnFault) {
                                    (*joints)[index].stopIssuedOnFault = true;
                                    needIssueStop = true;
                                }
                            } else {
                                (*joints)[index].stopIssuedOnFault = false;
                            }
                        }

                        if (hasFault) {
                            if (anyFaultObserved != nullptr) {
                                *anyFaultObserved = true;
                            }
                            if (needIssueStop) {
                                if (!proto->Stop(prepared.motorId)) {
                                    ROS_WARN_THROTTLE(
                                        1.0,
                                        "[CanDriverHW] Joint '%s' has fault, auto Stop rejected on '%s'.",
                                        prepared.jointName.c_str(),
                                        group.canDevice.c_str());
                                } else {
                                    ROS_WARN_THROTTLE(
                                        1.0,
                                        "[CanDriverHW] Joint '%s' has fault, auto Stop sent and motion command blocked.",
                                        prepared.jointName.c_str());
                                }
                            }
                            continue;
                        }
                    }

                    const bool enabled = hasSharedMotionStatus
                                             ? motionStatusSnapshots[index].enabled
                                             : proto->isEnabled(prepared.motorId);
                    if (config.safetyRequireEnabledForMotion && !enabled) {
                        ROS_WARN_THROTTLE(
                            1.0,
                            "[CanDriverHW] Joint '%s' is not enabled, motion command blocked.",
                            prepared.jointName.c_str());
                        continue;
                    }

                    if (prepared.route == PreparedCommandRoute::Velocity) {
                        if (!proto->setVelocity(prepared.motorId, rawCommandBuffer[index])) {
                            ROS_WARN_THROTTLE(
                                1.0,
                                "[CanDriverHW] setVelocity rejected on '%s'.",
                                group.canDevice.c_str());
                        }
                    } else if (prepared.route == PreparedCommandRoute::Csp) {
                        if (!proto->quickSetPosition(prepared.motorId, rawCommandBuffer[index])) {
                            ROS_WARN_THROTTLE(
                                1.0,
                                "[CanDriverHW] quickSetPosition rejected on '%s'.",
                                group.canDevice.c_str());
                        }
                    } else {
                        if (!proto->setPosition(prepared.motorId, rawCommandBuffer[index])) {
                            ROS_WARN_THROTTLE(
                                1.0,
                                "[CanDriverHW] setPosition rejected on '%s'.",
                                group.canDevice.c_str());
                        }
                    }
                } catch (const std::exception &e) {
                    ROS_ERROR_THROTTLE(1.0,
                                       "[CanDriverHW] write() command failed on '%s': %s",
                                       group.canDevice.c_str(),
                                       e.what());
                } catch (...) {
                    ROS_ERROR_THROTTLE(
                        1.0,
                        "[CanDriverHW] write() command failed on '%s' (unknown exception).",
                        group.canDevice.c_str());
                }
            }
        }
    }

    static PublishResult BuildMotorStateMessages(
        const IDeviceManager &deviceManager,
        const std::vector<CanDriverDeviceProtocolGroup> &groups,
        const std::deque<CanDriverJointConfig> &joints,
        std::mutex *jointStateMutex,
        std::int64_t feedbackFreshnessTimeoutNs =
            AxisReadinessEvaluator::Config{}.feedbackFreshnessTimeoutNs)
    {
        PublishResult result;
        result.messages.reserve(joints.size());

        struct StatusSnapshot {
            std::int32_t position{0};
            std::int32_t velocity{0};
            std::int16_t current{0};
            bool positionValid{false};
            bool velocityValid{false};
            bool currentValid{false};
            bool enabled{false};
            bool fault{false};
            std::uint8_t mode{can_driver::MotorState::MODE_UNKNOWN};
            bool statusValid{false};
            bool modeValid{false};
            bool feedbackFresh{false};
        };

        std::vector<StatusSnapshot> statusSnapshots(joints.size());
        const auto sharedState = deviceManager.getSharedDriverState();
        const auto nowNs = SharedDriverSteadyNowNs();
        if (sharedState) {
            for (const auto &group : groups) {
                for (const std::size_t index : group.jointIndices) {
                    const auto &joint = joints[index];
                    can_driver::SharedDriverState::AxisFeedbackState feedback;
                    if (!sharedState->getAxisFeedback(
                            can_driver::MakeAxisKey(group.canDevice, group.protocol, joint.motorId),
                            &feedback)) {
                        continue;
                    }

                    auto &snapshot = statusSnapshots[index];
                    snapshot.position = can_driver::safe_command::clampToInt32(feedback.position);
                    snapshot.velocity = can_driver::safe_command::clampToInt32(feedback.velocity);
                    snapshot.current = can_driver::safe_command::clampToInt16(feedback.current);
                    snapshot.positionValid = feedback.positionValid;
                    snapshot.velocityValid = feedback.velocityValid;
                    snapshot.currentValid = feedback.currentValid;
                    snapshot.enabled = feedback.enabled;
                    snapshot.fault = feedback.fault;
                    snapshot.mode = feedback.modeValid
                                        ? motorStateModeFromProtocolMode(feedback.mode)
                                        : can_driver::MotorState::MODE_UNKNOWN;
                    snapshot.statusValid = feedback.enabledValid && feedback.faultValid;
                    snapshot.modeValid = feedback.modeValid;
                    snapshot.feedbackFresh = sharedFeedbackFresh(
                        feedback, nowNs, feedbackFreshnessTimeoutNs);
                }
            }
        }

        std::lock_guard<std::mutex> lock(*jointStateMutex);
        for (std::size_t index = 0; index < joints.size(); ++index) {
            const auto &joint = joints[index];
            const auto &snapshot = statusSnapshots[index];
            can_driver::MotorState message;
            message.motor_id = static_cast<uint16_t>(joint.motorId);
            message.name = joint.name;

            message.position = snapshot.position;
            message.velocity = snapshot.velocity;
            message.current = snapshot.current;
            message.mode =
                snapshot.modeValid ? snapshot.mode : can_driver::MotorState::MODE_UNKNOWN;
            if (snapshot.statusValid) {
                message.enabled = snapshot.enabled;
                message.fault = snapshot.fault;
            }
            message.mode_valid = snapshot.modeValid;
            message.status_valid = snapshot.statusValid;
            message.position_valid = snapshot.positionValid;
            message.velocity_valid = snapshot.velocityValid;
            message.current_valid = snapshot.currentValid;
            message.feedback_fresh = snapshot.feedbackFresh;
            result.anyFault = result.anyFault || (message.status_valid && message.fault);

            result.messages.push_back(std::move(message));
        }

        return result;
    }

private:
    static std::uint8_t motorStateModeFromProtocolMode(CanProtocol::MotorMode mode)
    {
        switch (mode) {
        case CanProtocol::MotorMode::Velocity:
            return can_driver::MotorState::MODE_VELOCITY;
        case CanProtocol::MotorMode::CSP:
            return can_driver::MotorState::MODE_CSP;
        case CanProtocol::MotorMode::Position:
            return can_driver::MotorState::MODE_POSITION;
        }
        return can_driver::MotorState::MODE_UNKNOWN;
    }

    static bool sharedFeedbackFresh(const SharedDriverState::AxisFeedbackState &feedback,
                                    std::int64_t nowNs,
                                    std::int64_t feedbackFreshnessTimeoutNs =
                                        AxisReadinessEvaluator::Config{}.feedbackFreshnessTimeoutNs)
    {
        if (!feedback.feedbackSeen || feedback.lastRxSteadyNs <= 0) {
            return false;
        }

        if (feedbackFreshnessTimeoutNs <= 0 || nowNs <= feedback.lastRxSteadyNs) {
            return true;
        }
        return (nowNs - feedback.lastRxSteadyNs) <= feedbackFreshnessTimeoutNs;
    }

    static double clampWithJointLimits(const CanDriverJointConfig &joint, double cmdValue)
    {
        if (!joint.hasLimits || !std::isfinite(cmdValue)) {
            return cmdValue;
        }
        if (can_driver::controlModeUsesVelocitySemantics(joint.controlMode) &&
            joint.limits.has_velocity_limits) {
            return std::clamp(cmdValue,
                              -joint.limits.max_velocity,
                              joint.limits.max_velocity);
        }
        if (can_driver::controlModeUsesPositionSemantics(joint.controlMode) &&
            joint.limits.has_position_limits) {
            return std::clamp(cmdValue,
                              joint.limits.min_position,
                              joint.limits.max_position);
        }
        return cmdValue;
    }

    static void clearGroupCommands(const CanDriverDeviceProtocolGroup &group,
                                   std::deque<CanDriverJointConfig> *joints,
                                   std::vector<uint8_t> *commandValidBuffer,
                                   std::mutex *jointStateMutex)
    {
        std::lock_guard<std::mutex> lock(*jointStateMutex);
        for (const std::size_t index : group.jointIndices) {
            auto &joint = (*joints)[index];
            joint.hasDirectPosCmd = false;
            joint.hasDirectVelCmd = false;
            joint.stopIssuedOnFault = false;
            if (can_driver::controlModeUsesVelocitySemantics(joint.controlMode)) {
                joint.velCmd = 0.0;
            } else {
                joint.posCmd = joint.pos;
            }
            (*commandValidBuffer)[index] = 0;
        }
    }
};

} // namespace can_driver

#endif // CAN_DRIVER_CAN_DRIVER_IO_RUNTIME_H
