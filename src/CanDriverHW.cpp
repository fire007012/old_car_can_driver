#include "can_driver/AxisReadinessEvaluator.h"
#include "can_driver/AxisCommandSemantics.h"
#include "can_driver/CanDriverHW.h"
#include "can_driver/CanDriverIoRuntime.h"
#include "can_driver/InnfosEcbProtocol.h"
#include "can_driver/SafeCommand.h"
#include "can_driver/driver_ros_endpoints.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <tuple>
#include <unordered_map>
#include <xmlrpcpp/XmlRpcValue.h>

namespace {

using can_driver::toProtocolNodeId;

long long steadyAgeMs(std::int64_t stampNs)
{
    if (stampNs <= 0) {
        return -1;
    }
    const auto nowNs = can_driver::SharedDriverSteadyNowNs();
    return (nowNs > stampNs) ? static_cast<long long>((nowNs - stampNs) / 1000000) : 0;
}

constexpr double kDefaultPpVelocityRadS = (10.0 * 2.0 * M_PI / 60.0);

const char *protocolDisplayName(CanType protocol)
{
    switch (protocol) {
    case CanType::MT:
        return "MT";
    case CanType::PP:
        return "PP";
    case CanType::DM:
        return "DM";
    case CanType::ECB:
        return "ECB";
    }
    return "UNKNOWN";
}

bool sharedFeedbackFresh(const can_driver::SharedDriverState::AxisFeedbackState &feedback)
{
    if (!feedback.feedbackSeen || feedback.lastRxSteadyNs <= 0) {
        return false;
    }

    const auto nowNs = can_driver::SharedDriverSteadyNowNs();
    const can_driver::AxisReadinessEvaluator::Config config;
    if (config.feedbackFreshnessTimeoutNs <= 0 || nowNs <= feedback.lastRxSteadyNs) {
        return true;
    }
    return (nowNs - feedback.lastRxSteadyNs) <= config.feedbackFreshnessTimeoutNs;
}

} // namespace

CanDriverHW::CanDriverHW()
    : runtime_(),
      deviceManager_(runtime_.deviceManager()),
      motorActionExecutor_(runtime_.motorActionExecutor()),
      lifecycleDriverOps_(runtime_.lifecycleDriverOps()),
      commandGate_(runtime_.commandGate()),
      active_(runtime_.activeFlag()),
      lifecycleCoordinator_(runtime_.lifecycleCoordinator()),
      deviceLoopbackByName_(runtime_.deviceLoopbackByName())
{
    configureCommandGate();
    configureLifecycleCoordinator();
}

CanDriverHW::CanDriverHW(std::shared_ptr<IDeviceManager> deviceManager)
    : runtime_(std::move(deviceManager)),
      deviceManager_(runtime_.deviceManager()),
      motorActionExecutor_(runtime_.motorActionExecutor()),
      lifecycleDriverOps_(runtime_.lifecycleDriverOps()),
      commandGate_(runtime_.commandGate()),
      active_(runtime_.activeFlag()),
      lifecycleCoordinator_(runtime_.lifecycleCoordinator()),
      deviceLoopbackByName_(runtime_.deviceLoopbackByName())
{
    configureCommandGate();
    configureLifecycleCoordinator();
}

// ---------------------------------------------------------------------------
// 析构
// ---------------------------------------------------------------------------
CanDriverHW::~CanDriverHW()
{
    resetInternalState();
}

// ---------------------------------------------------------------------------
// init
// ---------------------------------------------------------------------------
bool CanDriverHW::init(ros::NodeHandle &nh, ros::NodeHandle &pnh)
{
    return init(nh, pnh, InitOptions{});
}

bool CanDriverHW::init(ros::NodeHandle &nh,
                       ros::NodeHandle &pnh,
                       const InitOptions &options)
{
    (void)nh;
    resetInternalState();
    warnOnDuplicateMotorIds_ = options.warn_on_duplicate_motor_ids;
    if (!loadRuntimeParams(pnh)) {
        return false;
    }
    if (!parseAndSetupJoints(pnh)) {
        resetInternalState();
        return false;
    }
    registerJointInterfaces();
    loadJointLimits(pnh);
    if (options.enable_ros_endpoints) {
        rosEndpoints_ = std::make_unique<DriverRosEndpoints>(*this, pnh);
    }

    std::set<std::string> configuredDevices;
    for (const auto &jc : joints_) {
        configuredDevices.insert(jc.canDevice);
    }

    ROS_INFO("[CanDriverHW] Initialized with %zu joints on %zu configured CAN device(s).",
             joints_.size(), configuredDevices.size());
    lifecycleCoordinator_.SetConfigured();
    if (rosEndpoints_) {
        rosEndpoints_->publishLifecycleStateNow();
    }
    return true;
}

void CanDriverHW::resetInternalState()
{
    if (rosEndpoints_) {
        rosEndpoints_->shutdown();
        rosEndpoints_.reset();
    }

    joints_.clear();
    jointIndexByName_.clear();
    jointGroups_.clear();
    rawCommandBuffer_.clear();
    commandValidBuffer_.clear();
    preparedCommandBuffer_.clear();
    jointZeroOffsetRadByMotorId_.clear();
    runtime_.reset();
}

bool CanDriverHW::loadRuntimeParams(const ros::NodeHandle &pnh)
{
    if (!pnh.getParam("direct_cmd_timeout_sec", directCmdTimeoutSec_)) {
        directCmdTimeoutSec_ = 0.5;
    }
    if (!std::isfinite(directCmdTimeoutSec_) || directCmdTimeoutSec_ < 0.0) {
        ROS_WARN("[CanDriverHW] Invalid direct_cmd_timeout_sec=%.9g, fallback to 0.5s.",
                 directCmdTimeoutSec_);
        directCmdTimeoutSec_ = 0.5;
    }

    if (!pnh.getParam("motor_state_period_sec", statePublishPeriodSec_)) {
        statePublishPeriodSec_ = 0.1;
    }
    if (!std::isfinite(statePublishPeriodSec_) || statePublishPeriodSec_ <= 0.0) {
        ROS_WARN("[CanDriverHW] Invalid motor_state_period_sec=%.9g, fallback to 0.1s.",
                 statePublishPeriodSec_);
        statePublishPeriodSec_ = 0.1;
    }

    if (!pnh.getParam("motor_query_hz", motorQueryHz_)) {
        motorQueryHz_ = 0.0;
    }
    if (!std::isfinite(motorQueryHz_)) {
        ROS_WARN("[CanDriverHW] Invalid motor_query_hz=%.9g, fallback to auto strategy.",
                 motorQueryHz_);
        motorQueryHz_ = 0.0;
    }

    if (!pnh.getParam("direct_cmd_queue_size", directCmdQueueSize_)) {
        directCmdQueueSize_ = 1;
    }
    if (directCmdQueueSize_ <= 0) {
        ROS_WARN("[CanDriverHW] Invalid direct_cmd_queue_size=%d, fallback to 1.",
                 directCmdQueueSize_);
        directCmdQueueSize_ = 1;
    }

    if (!pnh.getParam("debug_bypass_ros_control", debugBypassRosControl_)) {
        debugBypassRosControl_ = false;
    }
    if (!pnh.getParam("startup_position_sync_timeout_sec", startupPositionSyncTimeoutSec_)) {
        startupPositionSyncTimeoutSec_ = 1.0;
    }
    if (!std::isfinite(startupPositionSyncTimeoutSec_) || startupPositionSyncTimeoutSec_ < 0.0) {
        ROS_WARN("[CanDriverHW] Invalid startup_position_sync_timeout_sec=%.9g, fallback to 1.0s.",
                 startupPositionSyncTimeoutSec_);
        startupPositionSyncTimeoutSec_ = 1.0;
    }
    if (!pnh.getParam("startup_probe_query_hz", startupProbeQueryHz_)) {
        startupProbeQueryHz_ = 20.0;
    }
    if (!std::isfinite(startupProbeQueryHz_) || startupProbeQueryHz_ <= 0.0) {
        ROS_WARN("[CanDriverHW] Invalid startup_probe_query_hz=%.9g, fallback to 20.0 Hz.",
                 startupProbeQueryHz_);
        startupProbeQueryHz_ = 20.0;
    }
    int mtCommunicationTimeoutOnInitMs = 0;
    if (!pnh.getParam("mt_communication_timeout_on_init_ms", mtCommunicationTimeoutOnInitMs)) {
        mtCommunicationTimeoutOnInitMs = 0;
    }
    if (mtCommunicationTimeoutOnInitMs < 0) {
        ROS_WARN("[CanDriverHW] Invalid mt_communication_timeout_on_init_ms=%d, fallback to 0 ms.",
                 mtCommunicationTimeoutOnInitMs);
        mtCommunicationTimeoutOnInitMs = 0;
    }
    mtCommunicationTimeoutOnInitMs_ = static_cast<uint32_t>(mtCommunicationTimeoutOnInitMs);
    if (!pnh.getParam("safety_feedback_freshness_timeout_sec",
                      safetyFeedbackFreshnessTimeoutSec_)) {
        safetyFeedbackFreshnessTimeoutSec_ = 0.5;
    }
    if (!std::isfinite(safetyFeedbackFreshnessTimeoutSec_) ||
        safetyFeedbackFreshnessTimeoutSec_ <= 0.0) {
        ROS_WARN("[CanDriverHW] Invalid safety_feedback_freshness_timeout_sec=%.9g, fallback to 0.5s.",
                 safetyFeedbackFreshnessTimeoutSec_);
        safetyFeedbackFreshnessTimeoutSec_ = 0.5;
    }
    if (!pnh.getParam("pp_fast_write_enabled", ppFastWriteEnabled_)) {
        ppFastWriteEnabled_ = false;
    }
    if (!pnh.getParam("pp_position_default_velocity_rad_s", ppPositionDefaultVelocityRadS_)) {
        ppPositionDefaultVelocityRadS_ = kDefaultPpVelocityRadS;
    }
    if (!std::isfinite(ppPositionDefaultVelocityRadS_) || ppPositionDefaultVelocityRadS_ <= 0.0) {
        ROS_WARN("[CanDriverHW] Invalid pp_position_default_velocity_rad_s=%.9g, fallback to %.6f rad/s.",
                 ppPositionDefaultVelocityRadS_,
                 kDefaultPpVelocityRadS);
        ppPositionDefaultVelocityRadS_ = kDefaultPpVelocityRadS;
    }
    if (!pnh.getParam("pp_csp_default_velocity_rad_s", ppCspDefaultVelocityRadS_)) {
        ppCspDefaultVelocityRadS_ = kDefaultPpVelocityRadS;
    }
    if (!std::isfinite(ppCspDefaultVelocityRadS_) || ppCspDefaultVelocityRadS_ <= 0.0) {
        ROS_WARN("[CanDriverHW] Invalid pp_csp_default_velocity_rad_s=%.9g, fallback to %.6f rad/s.",
                 ppCspDefaultVelocityRadS_,
                 kDefaultPpVelocityRadS);
        ppCspDefaultVelocityRadS_ = kDefaultPpVelocityRadS;
    }
    if (!pnh.getParam("safety_stop_on_fault", safetyStopOnFault_)) {
        safetyStopOnFault_ = true;
    }
    if (!pnh.getParam("safety_require_enabled_for_motion", safetyRequireEnabledForMotion_)) {
        safetyRequireEnabledForMotion_ = true;
    }
    if (!pnh.getParam("lifecycle_require_enabled_for_running",
                      lifecycleRequireEnabledForRunning_)) {
        lifecycleRequireEnabledForRunning_ = true;
    }
    if (!pnh.getParam("max_position_step_rad", maxPositionStepRad_)) {
        maxPositionStepRad_ = 0.0;
    }
    if (!std::isfinite(maxPositionStepRad_) || maxPositionStepRad_ < 0.0) {
        ROS_WARN("[CanDriverHW] Invalid max_position_step_rad=%.9g, fallback to 0(disabled).",
                 maxPositionStepRad_);
        maxPositionStepRad_ = 0.0;
    }
    if (!pnh.getParam("safety_hold_after_device_recover", safetyHoldAfterDeviceRecover_)) {
        safetyHoldAfterDeviceRecover_ = true;
    }
    if (!pnh.getParam("pp_local_zero_offset_persistence_enabled",
                      ppLocalZeroOffsetPersistenceEnabled_)) {
        ppLocalZeroOffsetPersistenceEnabled_ = false;
    }
    if (!pnh.getParam("pp_zero_offset_persist_on_apply_to_motor",
                      ppZeroOffsetPersistOnApplyToMotor_)) {
        ppZeroOffsetPersistOnApplyToMotor_ = true;
    }
    if (!pnh.getParam("pp_local_zero_offset_file", ppLocalZeroOffsetFilePath_) ||
        ppLocalZeroOffsetFilePath_.empty()) {
        ppLocalZeroOffsetFilePath_ = defaultLocalZeroOffsetFilePath();
    }

    jointZeroOffsetRadByMotorId_.clear();
    if (ppLocalZeroOffsetPersistenceEnabled_ && !loadPersistedLocalZeroOffsets()) {
        ROS_ERROR("[CanDriverHW] Failed to load local zero offset persistence file '%s'.",
                  ppLocalZeroOffsetFilePath_.c_str());
        return false;
    }

    deviceManager_->setPpFastWriteEnabled(ppFastWriteEnabled_);
    deviceManager_->setRefreshRateHz(motorQueryHz_);
    deviceManager_->setMtCommunicationTimeoutOnInitMs(mtCommunicationTimeoutOnInitMs_);
    lifecycleDriverOps_.setFeedbackFreshnessTimeoutNs(
        static_cast<std::int64_t>(safetyFeedbackFreshnessTimeoutSec_ * 1e9));

    if (motorQueryHz_ > 0.0) {
        ROS_INFO("[CanDriverHW] motor_query_hz=%.3f Hz.", motorQueryHz_);
    }
    ROS_INFO("[CanDriverHW] pp_fast_write_enabled=%s.",
             ppFastWriteEnabled_ ? "true" : "false");
    ROS_INFO("[CanDriverHW] pp_position_default_velocity_rad_s=%.6f.",
             ppPositionDefaultVelocityRadS_);
    ROS_INFO("[CanDriverHW] pp_csp_default_velocity_rad_s=%.6f.",
             ppCspDefaultVelocityRadS_);
    ROS_INFO("[CanDriverHW] startup_position_sync_timeout_sec=%.3f s.",
             startupPositionSyncTimeoutSec_);
    ROS_INFO("[CanDriverHW] startup_probe_query_hz=%.3f Hz.",
             startupProbeQueryHz_);
    ROS_INFO("[CanDriverHW] mt_communication_timeout_on_init_ms=%u.",
             mtCommunicationTimeoutOnInitMs_);
    ROS_INFO("[CanDriverHW] safety_feedback_freshness_timeout_sec=%.3f s.",
             safetyFeedbackFreshnessTimeoutSec_);
    ROS_INFO("[CanDriverHW] safety_stop_on_fault=%s, safety_require_enabled_for_motion=%s, "
             "lifecycle_require_enabled_for_running=%s, max_position_step_rad=%.6f.",
             safetyStopOnFault_ ? "true" : "false",
             safetyRequireEnabledForMotion_ ? "true" : "false",
             lifecycleRequireEnabledForRunning_ ? "true" : "false",
             maxPositionStepRad_);
    ROS_INFO("[CanDriverHW] safety_hold_after_device_recover=%s.",
             safetyHoldAfterDeviceRecover_ ? "true" : "false");
    ROS_INFO("[CanDriverHW] pp_local_zero_offset_persistence_enabled=%s, file='%s'.",
             ppLocalZeroOffsetPersistenceEnabled_ ? "true" : "false",
             ppLocalZeroOffsetFilePath_.c_str());
    ROS_INFO("[CanDriverHW] pp_zero_offset_persist_on_apply_to_motor=%s.",
             ppZeroOffsetPersistOnApplyToMotor_ ? "true" : "false");
    ROS_WARN_STREAM_COND(debugBypassRosControl_,
                         "[CanDriverHW] debug_bypass_ros_control=true: "
                         "direct topic commands will bypass ros_control fallback.");
    return true;
}

bool CanDriverHW::syncStartupPositionAndCommands(const std::string &deviceFilter)
{
    struct JointSnapshot {
        double pos{0.0};
        double vel{0.0};
        double eff{0.0};
        bool valid{false};
    };

    std::vector<JointSnapshot> snapshots(joints_.size());
    std::vector<std::size_t> targetJointIndices;
    targetJointIndices.reserve(joints_.size());
    for (std::size_t i = 0; i < joints_.size(); ++i) {
        if (!deviceFilter.empty() && joints_[i].canDevice != deviceFilter) {
            continue;
        }
        targetJointIndices.push_back(i);
    }
    if (targetJointIndices.empty()) {
        return true;
    }

    // 给协议刷新线程一个短暂窗口拉取首轮真实反馈。
    const double timeout = startupPositionSyncTimeoutSec_;
    const auto sleepDur = std::chrono::milliseconds(20);
    const int maxPasses = std::max(1, static_cast<int>(std::ceil(timeout / 0.02)));
    const auto sharedState = deviceManager_ ? deviceManager_->getSharedDriverState() : nullptr;

    if (sharedState) {
        bool allValid = false;
        std::vector<std::string> missingJoints;
        for (int pass = 0; pass < maxPasses; ++pass) {
            allValid = true;
            missingJoints.clear();

            for (const std::size_t i : targetJointIndices) {
                const auto &jc = joints_[i];
                const auto axisMode =
                    can_driver::axisControlModeFromString(jc.controlMode);
                const bool needsPositionFeedback =
                    can_driver::controlModeUsesPositionSemantics(axisMode);
                can_driver::SharedDriverState::AxisFeedbackState feedback;
                const auto axisKey =
                    can_driver::MakeAxisKey(jc.canDevice, jc.protocol, jc.motorId);
                const bool hasFeedback = sharedState->getAxisFeedback(axisKey, &feedback);
                const bool hasFreshFeedback =
                    hasFeedback && feedback.feedbackSeen && feedback.lastRxSteadyNs > 0;
                const bool hasRequiredStartupState =
                    needsPositionFeedback
                        ? (hasFeedback && feedback.positionValid)
                        : (hasFeedback &&
                           (feedback.velocityValid || feedback.enabledValid ||
                            feedback.faultValid || feedback.currentValid ||
                            feedback.positionValid));
                if (!hasFreshFeedback || !hasRequiredStartupState) {
                    allValid = false;
                    missingJoints.push_back(
                        jc.name + (needsPositionFeedback ? "(position)" : "(velocity/state)"));
                    continue;
                }

                snapshots[i].pos = feedback.positionValid
                                       ? static_cast<double>(feedback.position) *
                                             can_driver::effectivePositionScale(jc)
                                       : 0.0;
                snapshots[i].vel = feedback.velocityValid
                                       ? static_cast<double>(feedback.velocity) *
                                             can_driver::effectiveVelocityScale(jc)
                                       : 0.0;
                snapshots[i].eff =
                    feedback.currentValid ? static_cast<double>(feedback.current) : 0.0;
                snapshots[i].valid = true;
            }

            if (allValid) {
                break;
            }

            if (pass + 1 < maxPasses) {
                std::this_thread::sleep_for(sleepDur);
            }
        }

        if (!allValid) {
            std::ostringstream oss;
            for (std::size_t i = 0; i < missingJoints.size(); ++i) {
                if (i > 0) {
                    oss << ", ";
                }
                oss << missingJoints[i];
            }
            if (deviceFilter.empty()) {
                ROS_ERROR("[CanDriverHW] Startup feedback sync timed out within %.3f s. "
                          "Missing required startup feedback for joints: %s",
                          timeout,
                          oss.str().c_str());
            } else {
                ROS_ERROR("[CanDriverHW] Startup feedback sync timed out on device '%s' within %.3f s. "
                          "Missing required startup feedback for joints: %s",
                          deviceFilter.c_str(),
                          timeout,
                          oss.str().c_str());
            }

            for (const std::size_t i : targetJointIndices) {
                const auto &jc = joints_[i];
                can_driver::SharedDriverState::AxisFeedbackState feedback;
                const auto axisKey =
                    can_driver::MakeAxisKey(jc.canDevice, jc.protocol, jc.motorId);
                const bool hasFeedback = sharedState->getAxisFeedback(axisKey, &feedback);
                if (!hasFeedback) {
                    ROS_ERROR("[CanDriverHW] Startup sync detail: joint '%s' has no shared feedback entry yet "
                              "(device=%s protocol=%s motor_id=%u).",
                              jc.name.c_str(),
                              jc.canDevice.c_str(),
                              protocolDisplayName(jc.protocol),
                              static_cast<unsigned>(static_cast<std::uint16_t>(jc.motorId)));
                    continue;
                }

                ROS_ERROR("[CanDriverHW] Startup sync detail: joint '%s' feedbackSeen=%s "
                          "positionValid=%s velocityValid=%s currentValid=%s enabled=%s "
                          "fault=%s timeoutCount=%u lastRxAgeMs=%lld rawPos=%lld rawVel=%d rawCur=%d mode=%d",
                          jc.name.c_str(),
                          feedback.feedbackSeen ? "true" : "false",
                          feedback.positionValid ? "true" : "false",
                          feedback.velocityValid ? "true" : "false",
                          feedback.currentValid ? "true" : "false",
                          feedback.enabled ? "true" : "false",
                          feedback.fault ? "true" : "false",
                          static_cast<unsigned>(feedback.consecutiveTimeoutCount),
                          steadyAgeMs(feedback.lastRxSteadyNs),
                          static_cast<long long>(feedback.position),
                          static_cast<int>(feedback.velocity),
                          static_cast<int>(feedback.current),
                          static_cast<int>(feedback.mode));
                }

            if (const auto concreteDeviceManager =
                    std::dynamic_pointer_cast<DeviceManager>(deviceManager_)) {
                std::set<std::string> devicesToReport;
                if (!deviceFilter.empty()) {
                    devicesToReport.insert(deviceFilter);
                } else {
                    for (const std::size_t i : targetJointIndices) {
                        devicesToReport.insert(joints_[i].canDevice);
                    }
                }

                for (const auto &device : devicesToReport) {
                    if (device.rfind("ecb://", 0) == 0) {
                        ROS_ERROR("[CanDriverHW] Startup sync ECB detail: device '%s' uses the Innfos SDK path "
                                  "and has no SocketCAN/Udp transport instance; check SDK discovery, motor_id, "
                                  "ECB IP, network interface, and logtool permissions.",
                                  device.c_str());
                        continue;
                    }

                    const auto transport = concreteDeviceManager->getTransport(device);
                    if (!transport) {
                        ROS_ERROR("[CanDriverHW] Startup sync transport detail: device '%s' has no transport instance.",
                                  device.c_str());
                        continue;
                    }

                    const auto stats = transport->snapshotStats();
                    ROS_ERROR("[CanDriverHW] Startup sync transport detail on '%s': tx_ok=%llu "
                              "tx_backpressure=%llu tx_link_down=%llu tx_error=%llu "
                              "rx_ok=%llu rx_error=%llu rx_short=%llu last_rx_age_ms=%lld",
                              device.c_str(),
                              static_cast<unsigned long long>(stats.txOk),
                              static_cast<unsigned long long>(stats.txBackpressure),
                              static_cast<unsigned long long>(stats.txLinkUnavailable),
                              static_cast<unsigned long long>(stats.txError),
                              static_cast<unsigned long long>(stats.rxOk),
                              static_cast<unsigned long long>(stats.rxError),
                              static_cast<unsigned long long>(stats.rxShortRead),
                              steadyAgeMs(stats.lastRxSteadyNs));
                }
            }
            return false;
        }
    } else {
        ROS_WARN("[CanDriverHW] Shared driver state unavailable during startup sync; "
                 "falling back to cached protocol values.");
        for (int pass = 0; pass < maxPasses; ++pass) {
            for (const auto &group : jointGroups_) {
                if (!deviceFilter.empty() && group.canDevice != deviceFilter) {
                    continue;
                }
                auto proto = getProtocol(group.canDevice, group.protocol);
                auto devMutex = getDeviceMutex(group.canDevice);
                if (!proto || !devMutex) {
                    continue;
                }

                std::lock_guard<std::mutex> devLock(*devMutex);
                for (const std::size_t i : group.jointIndices) {
                    const auto &jc = joints_[i];
                    snapshots[i].pos =
                        static_cast<double>(proto->getPosition(jc.motorId)) *
                        can_driver::effectivePositionScale(jc);
                    snapshots[i].vel =
                        static_cast<double>(proto->getVelocity(jc.motorId)) *
                        can_driver::effectiveVelocityScale(jc);
                    snapshots[i].eff =
                        static_cast<double>(proto->getCurrent(jc.motorId));
                    snapshots[i].valid = true;
                }
            }
            if (pass + 1 < maxPasses) {
                std::this_thread::sleep_for(sleepDur);
            }
        }
    }

    bool startupOutOfRange = false;
    {
        std::lock_guard<std::mutex> lock(jointStateMutex_);
        for (std::size_t i = 0; i < joints_.size(); ++i) {
            auto &jc = joints_[i];
            if (!deviceFilter.empty() && jc.canDevice != deviceFilter) {
                continue;
            }
            if (snapshots[i].valid) {
                jc.pos = snapshots[i].pos;
                jc.vel = snapshots[i].vel;
                jc.eff = snapshots[i].eff;
            }

            if (can_driver::controlModeUsesPositionSemantics(jc.controlMode)) {
                // 上电后将位置命令对齐到当前反馈，避免控制循环首拍跳变。
                // CSP 模式与 position 模式共用 posCmd，同样需要对齐。
                jc.posCmd = jc.pos;
                if (jc.hasLimits && jc.limits.has_position_limits) {
                    if (jc.pos < jc.limits.min_position || jc.pos > jc.limits.max_position) {
                        startupOutOfRange = true;
                        ROS_ERROR("[CanDriverHW] Joint '%s' startup position %.6f rad out of limits [%.6f, %.6f].",
                                  jc.name.c_str(),
                                  jc.pos,
                                  jc.limits.min_position,
                                  jc.limits.max_position);
                    }
                }
            } else {
                // 速度关节上电默认零速度命令。
                jc.velCmd = 0.0;
            }

            jc.hasDirectPosCmd = false;
            jc.hasDirectVelCmd = false;
            jc.stopIssuedOnFault = false;
        }
    }

    if (startupOutOfRange) {
        ROS_ERROR("[CanDriverHW] Startup position check failed. Refusing to activate to avoid limit violation.");
        return false;
    }

    if (deviceFilter.empty()) {
        ROS_INFO("[CanDriverHW] Startup position sync finished.");
    } else {
        ROS_INFO("[CanDriverHW] Startup position sync finished for device '%s'.",
                 deviceFilter.c_str());
    }
    return true;
}

bool CanDriverHW::parseAndSetupJoints(const ros::NodeHandle &pnh)
{
    XmlRpc::XmlRpcValue jointList;
    if (!pnh.getParam("joints", jointList)) {
        ROS_ERROR("[CanDriverHW] Parameter 'joints' not found under %s",
                  pnh.getNamespace().c_str());
        return false;
    }
    std::vector<joint_config_parser::ParsedJointConfig> parsed;
    std::string errorMsg;
    if (!joint_config_parser::parse(jointList, parsed, errorMsg)) {
        ROS_ERROR("[CanDriverHW] %s", errorMsg.c_str());
        return false;
    }

    std::set<std::string> seenJointNames;
    std::unordered_map<uint16_t, std::string> firstJointNameByMotorId;
    std::set<std::tuple<std::string, CanType, std::uint8_t>> seenProtocolNodes;
    bool hasDuplicateMotorId = false;
    for (const auto &p : parsed) {
        const std::string &jointName = p.name;
        const uint16_t motorId = static_cast<uint16_t>(p.motorId);
        const std::uint8_t protocolNodeId = toProtocolNodeId(p.motorId);

        if (!seenJointNames.insert(jointName).second) {
            ROS_ERROR("[CanDriverHW] Duplicate joint name '%s' in joints config.", jointName.c_str());
            return false;
        }
        const auto firstJointIt = firstJointNameByMotorId.find(motorId);
        if (firstJointIt != firstJointNameByMotorId.end()) {
            hasDuplicateMotorId = true;
            if (warnOnDuplicateMotorIds_) {
                ROS_WARN("[CanDriverHW] Duplicate motor_id=%u in joints config: '%s' and '%s'. "
                         "Startup will continue, but motor_id based maintenance services become ambiguous for this id.",
                         static_cast<unsigned>(motorId),
                         firstJointIt->second.c_str(),
                         jointName.c_str());
            }
        } else {
            firstJointNameByMotorId.emplace(motorId, jointName);
        }
        if (!seenProtocolNodes.emplace(p.canDevice, p.protocol, protocolNodeId).second) {
            ROS_ERROR("[CanDriverHW] Joint '%s' aliases protocol node id 0x%02X on device '%s' "
                      "protocol '%s'. Distinct system motor_id values must not collapse onto the "
                      "same on-wire node id.",
                      jointName.c_str(),
                      static_cast<unsigned>(protocolNodeId),
                      p.canDevice.c_str(),
                      protocolDisplayName(p.protocol));
            return false;
        }

        JointConfig jc;
        jc.name = p.name;
        jc.canDevice = p.canDevice;
        jc.controlMode = p.controlMode;
        jc.motorId = p.motorId;
        jc.protocol = p.protocol;
        jc.positionScale = p.positionScale;
        jc.velocityScale = p.velocityScale;
        jc.directionSign = p.directionSign;
        jc.ipMaxVelocity = p.ipMaxVelocity;
        jc.ipMaxAcceleration = p.ipMaxAcceleration;
        jc.ipMaxJerk = p.ipMaxJerk;
        jc.ipGoalTolerance = p.ipGoalTolerance;
        jc.ecbIp = p.ecbIp;
        jc.ecbAutoDiscovery = p.ecbAutoDiscovery;
        jc.ecbRefreshMs = p.ecbRefreshMs;
        jc.ecbProfilePositionMaxRpm = p.ecbProfilePositionMaxRpm;
        jc.ecbProfilePositionAccelerationRpmS = p.ecbProfilePositionAccelerationRpmS;
        jc.ecbProfilePositionDecelerationRpmS = p.ecbProfilePositionDecelerationRpmS;
        jc.ecbProfileVelocityAccelerationRpmS = p.ecbProfileVelocityAccelerationRpmS;
        jc.ecbProfileVelocityDecelerationRpmS = p.ecbProfileVelocityDecelerationRpmS;

        joints_.push_back(jc);
        jointIndexByName_[jc.name] = joints_.size() - 1;
    }

    if (hasDuplicateMotorId && warnOnDuplicateMotorIds_) {
        ROS_WARN("[CanDriverHW] Duplicate motor_id detected. "
                 "Robot HW init continues, but motor_id based service operations are disabled for ambiguous ids.");
    }

    rebuildJointGroups();
    rawCommandBuffer_.assign(joints_.size(), 0);
    commandValidBuffer_.assign(joints_.size(), 0);
    preparedCommandBuffer_.assign(joints_.size(), can_driver::CanDriverPreparedCommand{});
    syncLifecycleTargets();
    return true;
}

void CanDriverHW::rebuildJointGroups()
{
    std::map<std::pair<std::string, CanType>, std::vector<std::size_t>> groupedIndices;
    for (std::size_t i = 0; i < joints_.size(); ++i) {
        groupedIndices[{joints_[i].canDevice, joints_[i].protocol}].push_back(i);
    }

    jointGroups_.clear();
    jointGroups_.reserve(groupedIndices.size());
    for (const auto &entry : groupedIndices) {
        DeviceProtocolGroup group;
        group.canDevice = entry.first.first;
        group.protocol = entry.first.second;
        group.jointIndices = entry.second;
        jointGroups_.push_back(std::move(group));
    }
}

void CanDriverHW::registerJointInterfaces()
{
    for (auto &jc : joints_) {
        hardware_interface::JointStateHandle stateHandle(
            jc.name, &jc.pos, &jc.vel, &jc.eff);
        jntStateIface_.registerHandle(stateHandle);

        if (can_driver::controlModeUsesVelocitySemantics(jc.controlMode)) {
            hardware_interface::JointHandle velHandle(stateHandle, &jc.velCmd);
            velIface_.registerHandle(velHandle);
        } else {
            hardware_interface::JointHandle posHandle(stateHandle, &jc.posCmd);
            posIface_.registerHandle(posHandle);
        }
    }

    registerInterface(&jntStateIface_);
    registerInterface(&velIface_);
    registerInterface(&posIface_);
}

void CanDriverHW::loadJointLimits(const ros::NodeHandle &pnh)
{
    urdf::Model urdf;
    const bool urdfLoaded = urdf.initParam("robot_description");
    if (!urdfLoaded) {
        ROS_WARN("[CanDriverHW] Failed to load URDF from 'robot_description'. "
                 "Joint limits will not be enforced.");
    }

    for (auto &jc : joints_) {
        joint_limits_interface::JointLimits limits;
        joint_limits_interface::SoftJointLimits soft_limits;
        bool hasLimits = false;

        if (urdfLoaded) {
            urdf::JointConstSharedPtr urdfJoint = urdf.getJoint(jc.name);
            if (urdfJoint) {
                hasLimits = joint_limits_interface::getJointLimits(urdfJoint, limits);
                if (hasLimits) {
                    ROS_INFO("[CanDriverHW] Joint '%s': URDF limits [%.3f, %.3f] rad, "
                             "max_vel=%.3f rad/s, max_effort=%.3f",
                             jc.name.c_str(), limits.min_position, limits.max_position,
                             limits.max_velocity, limits.max_effort);
                }
            } else {
                ROS_WARN("[CanDriverHW] Joint '%s' not found in URDF.", jc.name.c_str());
            }
        }

        if (joint_limits_interface::getJointLimits(jc.name, pnh, limits)) {
            hasLimits = true;
            ROS_INFO("[CanDriverHW] Joint '%s': rosparam overrides limits.", jc.name.c_str());
        }
        joint_limits_interface::getSoftJointLimits(jc.name, pnh, soft_limits);

        if (hasLimits) {
            jc.limits = limits;
            jc.hasLimits = true;
            if (can_driver::controlModeUsesVelocitySemantics(jc.controlMode)) {
                joint_limits_interface::VelocityJointSaturationHandle handle(
                    velIface_.getHandle(jc.name), limits);
                velLimitsIface_.registerHandle(handle);
            } else {
                joint_limits_interface::PositionJointSaturationHandle handle(
                    posIface_.getHandle(jc.name), limits);
                posLimitsIface_.registerHandle(handle);
            }
        } else {
            ROS_WARN("[CanDriverHW] Joint '%s': no limits found, commands will not be clamped.",
                     jc.name.c_str());
        }
    }
}

void CanDriverHW::configureLifecycleCoordinator()
{
    runtime_.configureLifecycleCoordinator({
        [this]() {
            std::set<std::string> devices;
            for (const auto &joint : joints_) {
                devices.insert(joint.canDevice);
            }
            return std::vector<std::string>(devices.begin(), devices.end());
        },
        [this]() {
            std::lock_guard<std::mutex> stateLock(jointStateMutex_);
            for (auto &jc : joints_) {
                jc.hasDirectPosCmd = false;
                jc.hasDirectVelCmd = false;
                jc.stopIssuedOnFault = false;
            }
        },
        [this](std::string *detail) {
            return lifecycleDriverOps_.enableHealthy(detail);
        },
        [this](std::string *detail) {
            return lifecycleDriverOps_.motionHealthy(detail);
        },
        [this]() {
            return (motorQueryHz_ > 0.0)
                       ? std::min(startupProbeQueryHz_, motorQueryHz_)
                       : startupProbeQueryHz_;
        },
        [this](const std::string &device, double refreshRateHz) {
            deviceManager_->setDeviceRefreshRateHz(device, refreshRateHz);
        },
        [this](const std::string &device) {
            return applyPersistedPpZeroOffsets(device);
        },
        [this](const std::string &device) {
            return syncStartupPositionAndCommands(device);
        },
        [this](const std::string &device) {
            return applyPerAxisPpDefaultVelocities(device);
        },
        [this](const std::string &device) {
            return applyInitialModes(device);
        },
    });
}

void CanDriverHW::configureCommandGate()
{
    runtime_.configureCommandGate(
        [this]() {
            return captureCommandSnapshots();
        },
        [this]() {
            holdCommandsForLifecycleTransition();
        });
}

void CanDriverHW::configureMotorMaintenanceService(MotorMaintenanceService &service)
{
    service.configure(
        [this]() {
            return active_.load(std::memory_order_acquire);
        },
        &motorActionExecutor_,
        [this](uint16_t motorId, JointConfig *joint) {
            return lookupJointByMotorId(motorId, joint);
        },
        [this](const std::string &jointName) {
            clearDirectCommand(jointName);
        },
        [this](uint16_t motorId, can_driver::AxisControlMode mode) {
            return commitModeSwitch(motorId, mode);
        },
        [this](uint16_t motorId, double zeroOffset, double previousZeroOffset) {
            return commitZero(motorId, zeroOffset, previousZeroOffset);
        },
        [this](uint16_t motorId, double* zeroOffset) {
            return getZeroOffset(motorId, zeroOffset);
        },
        [this](uint16_t motorId, double baseMin, double baseMax, double zeroOffset) {
            return commitLimits(motorId, baseMin, baseMax, zeroOffset);
        },
        [this](const JointConfig &joint, can_driver::SharedDriverState::AxisFeedbackState *feedback) {
            return getFreshAxisFeedback(joint, feedback);
        },
        [this](const JointConfig &joint, const char *operation, std::string *message) {
            return requireAxisDisabledForConfiguration(joint, operation, message);
        },
        [this](const std::string &device, CanType type) {
            return getProtocol(device, type);
        },
        [this](const std::string &device) {
            return getDeviceMutex(device);
        },
        ppZeroOffsetPersistOnApplyToMotor_);
}

std::vector<CanDriverHW::DirectCommandEndpoint> CanDriverHW::directCommandEndpoints() const
{
    std::vector<DirectCommandEndpoint> endpoints;
    endpoints.reserve(joints_.size());
    for (std::size_t i = 0; i < joints_.size(); ++i) {
        endpoints.push_back(DirectCommandEndpoint{joints_[i].name, i});
    }
    return endpoints;
}

std::vector<CanDriverHW::JointRuntimeStateView> CanDriverHW::snapshotJointRuntimeStates() const
{
    std::vector<JointRuntimeStateView> result;
    result.reserve(joints_.size());

    const auto sharedState = deviceManager_ ? deviceManager_->getSharedDriverState() : nullptr;
    std::unordered_map<std::string, bool> deviceReadyByName;

    std::lock_guard<std::mutex> lock(jointStateMutex_);
    for (const auto& joint : joints_) {
        JointRuntimeStateView item;
        item.jointName = joint.name;
        item.controlMode = joint.controlMode;
        item.position = joint.pos;
        item.velocity = joint.vel;
        item.effort = joint.eff;

        const auto deviceReadyIt = deviceReadyByName.find(joint.canDevice);
        if (deviceReadyIt != deviceReadyByName.end()) {
            item.deviceReady = deviceReadyIt->second;
        } else {
            const bool deviceReady =
                deviceManager_ ? deviceManager_->isDeviceReady(joint.canDevice) : false;
            deviceReadyByName.emplace(joint.canDevice, deviceReady);
            item.deviceReady = deviceReady;
        }

        if (sharedState) {
            can_driver::SharedDriverState::AxisFeedbackState feedback;
            const auto axisKey =
                can_driver::MakeAxisKey(joint.canDevice, joint.protocol, joint.motorId);
            if (sharedState->getAxisFeedback(axisKey, &feedback)) {
                item.enabled = feedback.enabledValid && feedback.enabled;
                item.fault = feedback.faultValid && feedback.fault;
                item.feedbackFresh = sharedFeedbackFresh(feedback);
            }

            can_driver::SharedDriverState::AxisCommandState command;
            if (sharedState->getAxisCommand(axisKey, &command)) {
                item.commandValid = command.valid;
            }
        }

        result.push_back(std::move(item));
    }

    return result;
}

void CanDriverHW::acceptDirectCommand(std::size_t jointIndex,
                                      bool isVelocity,
                                      double value,
                                      const ros::Time &stamp)
{
    if (!active_.load(std::memory_order_acquire) || jointIndex >= joints_.size()) {
        return;
    }

    std::lock_guard<std::mutex> lock(jointStateMutex_);
    auto &jc = joints_[jointIndex];
    if (isVelocity) {
        jc.directVelCmd = value;
        jc.hasDirectVelCmd = true;
        jc.lastDirectVelTime = stamp;
    } else {
        jc.directPosCmd = value;
        jc.hasDirectPosCmd = true;
        jc.lastDirectPosTime = stamp;
    }
}

bool CanDriverHW::lookupJointByMotorId(uint16_t motorId,
                                       can_driver::CanDriverJointConfig *joint) const
{
    if (joint == nullptr) {
        return false;
    }
    const JointConfig *matched = nullptr;
    std::size_t matchCount = 0;
    for (const auto &candidate : joints_) {
        if (static_cast<uint16_t>(candidate.motorId) == motorId) {
            ++matchCount;
            if (matched == nullptr) {
                matched = &candidate;
            }
        }
    }
    if (matchCount == 1 && matched != nullptr) {
        *joint = *matched;
        return true;
    }
    if (matchCount > 1) {
        ROS_ERROR_THROTTLE(2.0,
                           "[CanDriverHW] motor_id=%u is ambiguous (%zu joints share this id). "
                           "Rejecting motor_id-based maintenance request.",
                           static_cast<unsigned>(motorId),
                           matchCount);
    }
    return false;
}

void CanDriverHW::clearDirectCommand(const std::string &jointName)
{
    std::lock_guard<std::mutex> lock(jointStateMutex_);
    const auto it = jointIndexByName_.find(jointName);
    if (it == jointIndexByName_.end()) {
        return;
    }
    joints_[it->second].hasDirectPosCmd = false;
    joints_[it->second].hasDirectVelCmd = false;
}

bool CanDriverHW::commitModeSwitch(uint16_t motorId, can_driver::AxisControlMode mode)
{
    std::lock_guard<std::mutex> lock(jointStateMutex_);
    std::size_t matchedIndex = joints_.size();
    std::size_t matchCount = 0;
    for (std::size_t i = 0; i < joints_.size(); ++i) {
        if (static_cast<uint16_t>(joints_[i].motorId) != motorId) {
            continue;
        }
        ++matchCount;
        if (matchedIndex == joints_.size()) {
            matchedIndex = i;
        }
    }
    if (matchCount != 1 || matchedIndex >= joints_.size()) {
        if (matchCount > 1) {
            ROS_ERROR("[CanDriverHW] Reject mode switch for ambiguous motor_id=%u (%zu matches).",
                      static_cast<unsigned>(motorId),
                      matchCount);
        }
        return false;
    }

    auto &joint = joints_[matchedIndex];
    joint.controlMode = can_driver::axisControlModeName(mode);
    joint.hasDirectPosCmd = false;
    joint.hasDirectVelCmd = false;
    joint.posCmd = joint.pos;
    joint.velCmd = 0.0;
    joint.requireCommandAlignment = true;
    commandValidBuffer_[matchedIndex] = 0;
    return true;
}

bool CanDriverHW::getZeroOffset(uint16_t motorId, double* zeroOffset) const
{
    if (zeroOffset == nullptr) {
        return false;
    }
    std::lock_guard<std::mutex> lock(jointStateMutex_);
    std::size_t matchCount = 0;
    for (const auto &joint : joints_) {
        if (static_cast<uint16_t>(joint.motorId) == motorId) {
            ++matchCount;
        }
    }
    if (matchCount > 1) {
        ROS_ERROR_THROTTLE(2.0,
                           "[CanDriverHW] Reject zero-offset query for ambiguous motor_id=%u (%zu matches).",
                           static_cast<unsigned>(motorId),
                           matchCount);
        return false;
    }
    const auto it = jointZeroOffsetRadByMotorId_.find(motorId);
    if (it == jointZeroOffsetRadByMotorId_.end()) {
        *zeroOffset = 0.0;
        return true;
    }
    *zeroOffset = it->second;
    return true;
}

bool CanDriverHW::commitZero(uint16_t motorId,
                             double zeroOffset,
                             double previousZeroOffset)
{
    std::lock_guard<std::mutex> lock(jointStateMutex_);
    std::size_t matchCount = 0;
    for (auto &joint : joints_) {
        if (static_cast<uint16_t>(joint.motorId) != motorId) {
            continue;
        }
        ++matchCount;
    }
    if (matchCount != 1) {
        if (matchCount > 1) {
            ROS_ERROR("[CanDriverHW] Reject zero-offset commit for ambiguous motor_id=%u (%zu matches).",
                      static_cast<unsigned>(motorId),
                      matchCount);
        }
        return false;
    }

    for (auto &joint : joints_) {
        if (static_cast<uint16_t>(joint.motorId) != motorId) {
            continue;
        }
        jointZeroOffsetRadByMotorId_[motorId] = zeroOffset;
        if (ppLocalZeroOffsetPersistenceEnabled_ && !savePersistedLocalZeroOffsets()) {
            jointZeroOffsetRadByMotorId_[motorId] = previousZeroOffset;
            return false;
        }
        return true;
    }
    return false;
}

bool CanDriverHW::commitLimits(uint16_t motorId,
                               double baseMin,
                               double baseMax,
                               double zeroOffset)
{
    std::lock_guard<std::mutex> lock(jointStateMutex_);
    std::size_t matchCount = 0;
    for (const auto &joint : joints_) {
        if (static_cast<uint16_t>(joint.motorId) == motorId) {
            ++matchCount;
        }
    }
    if (matchCount != 1) {
        if (matchCount > 1) {
            ROS_ERROR("[CanDriverHW] Reject limits commit for ambiguous motor_id=%u (%zu matches).",
                      static_cast<unsigned>(motorId),
                      matchCount);
        }
        return false;
    }

    for (auto &joint : joints_) {
        if (static_cast<uint16_t>(joint.motorId) != motorId) {
            continue;
        }
        joint.hasLimits = true;
        joint.limits.has_position_limits = true;
        joint.limits.min_position = baseMin;
        joint.limits.max_position = baseMax;
        jointZeroOffsetRadByMotorId_[motorId] = zeroOffset;
        return true;
    }
    return false;
}

bool CanDriverHW::applyPersistedPpZeroOffsets(const std::string &deviceFilter)
{
    if (!ppLocalZeroOffsetPersistenceEnabled_) {
        return true;
    }

    bool appliedAny = false;
    std::map<uint16_t, double> persistedOffsets;
    {
        std::lock_guard<std::mutex> lock(jointStateMutex_);
        persistedOffsets = jointZeroOffsetRadByMotorId_;
    }

    for (const auto &joint : joints_) {
        if (joint.protocol != CanType::PP ||
            !can_driver::controlModeUsesPositionSemantics(joint.controlMode)) {
            continue;
        }
        if (!deviceFilter.empty() && joint.canDevice != deviceFilter) {
            continue;
        }
        const auto it = persistedOffsets.find(static_cast<uint16_t>(joint.motorId));
        if (it == persistedOffsets.end()) {
            continue;
        }

        int32_t rawOffset = 0;
        if (!can_driver::safe_command::scaleAndClampToInt32(
                it->second,
                can_driver::effectivePositionScale(joint),
                joint.name + ".persisted_zero_offset",
                rawOffset)) {
            ROS_ERROR("[CanDriverHW] Failed to convert persisted zero offset for joint '%s'.",
                      joint.name.c_str());
            return false;
        }

        const auto status = motorActionExecutor_.execute(
            makeMotorTarget(joint),
            [rawOffset](const std::shared_ptr<CanProtocol> &proto, MotorID id) {
                return proto->setPositionOffset(id, rawOffset);
            },
            "Restore persisted zero offset");
        if (status != MotorActionExecutor::Status::Ok) {
            ROS_ERROR("[CanDriverHW] Failed to restore persisted zero offset for joint '%s'.",
                      joint.name.c_str());
            return false;
        }
        ROS_INFO("[CanDriverHW] Restored persisted zero offset %.6f rad for joint '%s'.",
                 it->second,
                 joint.name.c_str());
        appliedAny = true;
    }

    if (!appliedAny && deviceFilter.empty()) {
        ROS_INFO("[CanDriverHW] No persisted PP zero offsets to restore.");
    }
    return true;
}

std::string CanDriverHW::defaultLocalZeroOffsetFilePath() const
{
    const char *home = std::getenv("HOME");
    const std::filesystem::path root =
        (home != nullptr && home[0] != '\0') ? std::filesystem::path(home) / ".ros"
                                             : std::filesystem::temp_directory_path();
    return (root / "can_driver_pp_zero_offsets.txt").string();
}

bool CanDriverHW::loadPersistedLocalZeroOffsets()
{
    const std::filesystem::path path(ppLocalZeroOffsetFilePath_);
    if (!std::filesystem::exists(path)) {
        return true;
    }

    std::ifstream in(path);
    if (!in.is_open()) {
        return false;
    }

    std::map<uint16_t, double> loaded;
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#') {
            continue;
        }
        std::istringstream iss(line);
        uint16_t motorId = 0;
        double zeroOffset = 0.0;
        if (!(iss >> motorId >> zeroOffset)) {
            return false;
        }
        loaded[motorId] = zeroOffset;
    }

    jointZeroOffsetRadByMotorId_ = std::move(loaded);
    return true;
}

bool CanDriverHW::savePersistedLocalZeroOffsets() const
{
    const std::filesystem::path path(ppLocalZeroOffsetFilePath_);
    std::error_code ec;
    std::filesystem::create_directories(path.parent_path(), ec);
    if (ec) {
        return false;
    }

    const std::filesystem::path tmp = path.string() + ".tmp";
    std::ofstream out(tmp, std::ios::trunc);
    if (!out.is_open()) {
        return false;
    }
    out.setf(std::ios::fixed);
    out.precision(17);
    for (const auto &entry : jointZeroOffsetRadByMotorId_) {
        out << entry.first << ' ' << entry.second << '\n';
    }
    out.close();
    if (!out) {
        return false;
    }
    std::filesystem::rename(tmp, path, ec);
    if (ec) {
        return false;
    }
    return true;
}

void CanDriverHW::holdCommandsForLifecycleTransition()
{
    std::lock_guard<std::mutex> stateLock(jointStateMutex_);
    for (std::size_t i = 0; i < joints_.size(); ++i) {
        auto &jc = joints_[i];
        jc.hasDirectPosCmd = false;
        jc.hasDirectVelCmd = false;
        if (can_driver::controlModeUsesVelocitySemantics(jc.controlMode)) {
            jc.velCmd = 0.0;
        } else {
            jc.posCmd = jc.pos;
        }
        jc.requireCommandAlignment = false;
        commandValidBuffer_[i] = 0;
    }
}

std::vector<CommandGate::Snapshot> CanDriverHW::captureCommandSnapshots() const
{
    std::lock_guard<std::mutex> stateLock(jointStateMutex_);
    std::vector<CommandGate::Snapshot> snapshots(joints_.size());
    for (std::size_t i = 0; i < joints_.size(); ++i) {
        const auto &jc = joints_[i];
        const auto mode = can_driver::axisControlModeFromString(jc.controlMode);
        auto &snapshot = snapshots[i];
        snapshot.controlMode = mode;
        snapshot.commandValue = can_driver::controlModeSelectedCommandValue(jc, mode);
        snapshot.hasDirectCommand = can_driver::controlModeHasDirectCommand(jc, mode);
        snapshot.targetNearActual = can_driver::controlModeTargetNearActual(jc, mode);
    }
    return snapshots;
}

void CanDriverHW::syncLifecycleTargets()
{
    std::vector<MotorActionExecutor::Target> targets;
    targets.reserve(joints_.size());
    for (const auto &jc : joints_) {
        targets.push_back(MotorActionExecutor::Target{jc.name, jc.canDevice, jc.protocol, jc.motorId});
    }
    lifecycleDriverOps_.setTargets(std::move(targets));
}

MotorActionExecutor::Target CanDriverHW::makeMotorTarget(const JointConfig &jc) const
{
    return MotorActionExecutor::Target{jc.name, jc.canDevice, jc.protocol, jc.motorId};
}

bool CanDriverHW::applyInitialModes(const std::string &deviceFilter)
{
    bool allOk = true;
    for (const auto &jc : joints_) {
        if (!deviceFilter.empty() && jc.canDevice != deviceFilter) {
            continue;
        }
        if (can_driver::axisControlModeFromString(jc.controlMode) !=
            can_driver::AxisControlMode::Csp) {
            continue;
        }
        const auto status = motorActionExecutor_.execute(
            makeMotorTarget(jc),
            [](const std::shared_ptr<CanProtocol> &proto, MotorID id) {
                return proto->setMode(id, CanProtocol::MotorMode::CSP);
            },
            "Set initial CSP mode");
        if (status != MotorActionExecutor::Status::Ok) {
            ROS_ERROR("[CanDriverHW] applyInitialModes: setMode(CSP) failed for joint '%s'. "
                      "Refusing to activate to prevent motion in wrong mode.",
                      jc.name.c_str());
            allOk = false;
        } else {
            ROS_INFO("[CanDriverHW] applyInitialModes: joint '%s' set to CSP mode.",
                     jc.name.c_str());
        }
    }
    return allOk;
}

bool CanDriverHW::applyPerAxisPpDefaultVelocities(const std::string &deviceFilter)
{
    bool appliedAny = false;
    for (const auto &group : jointGroups_) {
        if (group.protocol != CanType::PP) {
            continue;
        }
        if (!deviceFilter.empty() && group.canDevice != deviceFilter) {
            continue;
        }

        auto protocol = std::dynamic_pointer_cast<EyouCan>(getProtocol(group.canDevice, group.protocol));
        if (!protocol) {
            ROS_WARN("[CanDriverHW] Skip per-axis PP default velocity override on '%s' because "
                     "the protocol instance is not EyouCan.",
                     group.canDevice.c_str());
            continue;
        }

        for (const auto jointIndex : group.jointIndices) {
            const auto &joint = joints_[jointIndex];
            int32_t rawVelocity = 0;
            if (!can_driver::safe_command::scaleAndClampToInt32(ppPositionDefaultVelocityRadS_,
                                                                joint.velocityScale,
                                                                joint.name + ".pp_position_default_velocity_rad_s",
                                                                rawVelocity)) {
                return false;
            }
            protocol->setMotorDefaultPositionVelocityRaw(joint.motorId, rawVelocity);

            if (!can_driver::safe_command::scaleAndClampToInt32(ppCspDefaultVelocityRadS_,
                                                                joint.velocityScale,
                                                                joint.name + ".pp_csp_default_velocity_rad_s",
                                                                rawVelocity)) {
                return false;
            }
            protocol->setMotorDefaultCspVelocityRaw(joint.motorId, rawVelocity);
            appliedAny = true;
        }
    }

    if (appliedAny) {
        ROS_INFO("[CanDriverHW] Applied per-axis PP default velocity overrides from rad/s configuration%s.",
                 deviceFilter.empty() ? "" : (" on " + deviceFilter).c_str());
    }
    return true;
}

// ---------------------------------------------------------------------------
// read
// ---------------------------------------------------------------------------
void CanDriverHW::read(const ros::Time & /*time*/, const ros::Duration & /*period*/)
{
    if (!active_.load(std::memory_order_acquire)) {
        return;
    }
    can_driver::CanDriverIoRuntime::SyncJointFeedback(
        *deviceManager_, jointGroups_, &joints_, &jointStateMutex_);
}

// ---------------------------------------------------------------------------
// write
// ---------------------------------------------------------------------------
void CanDriverHW::write(const ros::Time & /*time*/, const ros::Duration &period)
{
    if (!active_.load(std::memory_order_acquire)) {
        return;
    }

    if (lifecycleCoordinator_.mode() != can_driver::SystemOpMode::Running) {
        std::lock_guard<std::mutex> lock(jointStateMutex_);
        std::fill(commandValidBuffer_.begin(), commandValidBuffer_.end(), 0);
        return;
    }
    if (!commandGate_.consumeFreshCommandLatchIfSatisfied()) {
        std::lock_guard<std::mutex> lock(jointStateMutex_);
        std::fill(commandValidBuffer_.begin(), commandValidBuffer_.end(), 0);
        return;
    }

#if SOFTWARE_LOOPBACK_MODE
    // ========== 软件回环模式 ==========
    // 不发送 CAN 帧，命令值已经在 write() 被 ros_control 写入 posCmd/velCmd
    // read() 会直接读取这些值作为反馈
#else
    // ========== 真实 CAN 模式 ==========
    bool anyFaultObserved = false;

    // 应用关节限位（钳制命令值到安全范围）
    posLimitsIface_.enforceLimits(period);
    velLimitsIface_.enforceLimits(period);
    const can_driver::CanDriverIoRuntime::WriteConfig writeConfig{
        directCmdTimeoutSec_,
        debugBypassRosControl_,
        safetyStopOnFault_,
        safetyRequireEnabledForMotion_,
        maxPositionStepRad_,
        safetyHoldAfterDeviceRecover_,
    };
    can_driver::CanDriverIoRuntime::PrepareCommands(
        &joints_,
        &rawCommandBuffer_,
        &commandValidBuffer_,
        &preparedCommandBuffer_,
        &jointStateMutex_,
        writeConfig);
    can_driver::CanDriverIoRuntime::DispatchPreparedCommands(*deviceManager_,
                                                             jointGroups_,
                                                             &joints_,
                                                             rawCommandBuffer_,
                                                             &commandValidBuffer_,
                                                             preparedCommandBuffer_,
                                                             &jointStateMutex_,
                                                             &commandGate_,
                                                             writeConfig,
                                                             &anyFaultObserved);
    bool unhealthy = anyFaultObserved;
    std::string healthDetail;
    if (!unhealthy && !lifecycleHealthHealthy(&healthDetail)) {
        unhealthy = true;
        ROS_WARN_THROTTLE(1.0,
                          "[CanDriverHW] Auto-fault because lifecycle health check failed: %s",
                          healthDetail.empty() ? "unknown reason" : healthDetail.c_str());
    }
    lifecycleCoordinator_.UpdateFromFeedback(unhealthy);
#endif
}

// ---------------------------------------------------------------------------
// 内部辅助
// ---------------------------------------------------------------------------
bool CanDriverHW::initDevice(const std::string &device, bool loopback)
{
    std::vector<std::pair<CanType, MotorID>> motors;
    for (const auto &jc : joints_) {
        if (jc.canDevice != device) {
            continue;
        }
        motors.emplace_back(jc.protocol, jc.motorId);
    }
    const bool ok = deviceManager_->initDevice(device, motors, loopback);
    if (!ok) {
        return false;
    }

    for (const auto &jc : joints_) {
        if (jc.canDevice != device || jc.protocol != CanType::ECB) {
            continue;
        }

        const auto baseProto = deviceManager_->getProtocol(jc.canDevice, jc.protocol);
        const auto ecbProto = std::dynamic_pointer_cast<InnfosEcbProtocol>(baseProto);
        if (!ecbProto) {
            ROS_ERROR("[CanDriverHW] ECB protocol cast failed on '%s'.", jc.canDevice.c_str());
            return false;
        }
        ecbProto->configureMotorRouting(jc.motorId, jc.ecbIp, jc.ecbAutoDiscovery);
        ecbProto->configureMotionProfile(jc.motorId,
                                          jc.ecbProfilePositionMaxRpm,
                                          jc.ecbProfilePositionAccelerationRpmS,
                                          jc.ecbProfilePositionDecelerationRpmS,
                                          jc.ecbProfileVelocityAccelerationRpmS,
                                          jc.ecbProfileVelocityDecelerationRpmS);
        if (jc.ecbRefreshMs > 0) {
            ecbProto->setRefreshRateHz(1000.0 / static_cast<double>(jc.ecbRefreshMs));
        }
    }

    return true;
}

std::shared_ptr<CanProtocol> CanDriverHW::getProtocol(const std::string &device, CanType type) const
{
    return deviceManager_->getProtocol(device, type);
}

std::shared_ptr<std::mutex> CanDriverHW::getDeviceMutex(const std::string &device) const
{
    return deviceManager_->getDeviceMutex(device);
}

bool CanDriverHW::isDeviceReady(const std::string &device) const
{
    return deviceManager_->isDeviceReady(device);
}

bool CanDriverHW::getFreshAxisFeedback(
    const JointConfig &joint,
    can_driver::SharedDriverState::AxisFeedbackState *feedback) const
{
    if (feedback == nullptr || !deviceManager_) {
        return false;
    }

    const auto sharedState = deviceManager_->getSharedDriverState();
    if (!sharedState) {
        return false;
    }

    if (!sharedState->getAxisFeedback(
            can_driver::MakeAxisKey(joint.canDevice, joint.protocol, joint.motorId), feedback)) {
        return false;
    }

    return sharedFeedbackFresh(*feedback);
}

bool CanDriverHW::requireAxisDisabledForConfiguration(const JointConfig &joint,
                                                      const char *operation,
                                                      std::string *message) const
{
    can_driver::SharedDriverState::AxisFeedbackState feedback;
    if (!getFreshAxisFeedback(joint, &feedback) || !feedback.enabledValid) {
        if (message != nullptr) {
            *message = std::string(operation ? operation : "Configuration") +
                       " requires fresh enable-state feedback while the motor is disabled.";
        }
        return false;
    }

    if (feedback.enabled) {
        if (message != nullptr) {
            *message = std::string(operation ? operation : "Configuration") +
                       " requires the motor to be disabled first.";
        }
        return false;
    }

    return true;
}

bool CanDriverHW::lifecycleHealthHealthy(std::string *detail) const
{
    const auto mode = lifecycleCoordinator_.mode();
    if (mode == can_driver::SystemOpMode::Armed) {
        return lifecycleDriverOps_.enableHealthy(detail);
    }
    if (mode == can_driver::SystemOpMode::Running) {
        if (!lifecycleRequireEnabledForRunning_) {
            return lifecycleDriverOps_.enableHealthy(detail);
        }
        return lifecycleDriverOps_.motionHealthy(detail);
    }
    return true;
}

void CanDriverHW::publishMotorStates(ros::Publisher &publisher)
{
    if (!active_.load(std::memory_order_acquire)) {
        return;
    }
    auto publishResult = can_driver::CanDriverIoRuntime::BuildMotorStateMessages(
        *deviceManager_, jointGroups_, joints_, &jointStateMutex_);
    bool unhealthy = publishResult.anyFault;
    std::string healthDetail;
    if (!unhealthy && !lifecycleHealthHealthy(&healthDetail)) {
        unhealthy = true;
        ROS_WARN_THROTTLE(1.0,
                          "[CanDriverHW] Auto-fault because lifecycle health check failed: %s",
                          healthDetail.empty() ? "unknown reason" : healthDetail.c_str());
    }
    lifecycleCoordinator_.UpdateFromFeedback(unhealthy);
    if (!active_.load(std::memory_order_acquire)) {
        return;
    }
    for (const auto &msg : publishResult.messages) {
        publisher.publish(msg);
    }
}

void CanDriverHW::publishLifecycleState(ros::Publisher &publisher)
{
    if (!publisher) {
        return;
    }

    std_msgs::String msg;
    msg.data = can_driver::SystemOpModeName(lifecycleCoordinator_.mode());
    publisher.publish(msg);
}
