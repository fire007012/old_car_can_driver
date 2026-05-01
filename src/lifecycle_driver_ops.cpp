#include "can_driver/AxisReadinessEvaluator.h"
#include "can_driver/lifecycle_driver_ops.hpp"
#include "can_driver/SharedDriverState.h"

#include <algorithm>
#include <chrono>
#include <thread>
#include <utility>

namespace can_driver {

namespace {

const char *axisReadinessProtocolName(CanType protocol)
{
    if (protocol == CanType::MT) {
        return "mt";
    }
    if (protocol == CanType::PP) {
        return "pp";
    }
    if (protocol == CanType::DM) {
        return "dm";
    }
    return "ecb";
}

std::string axisReadinessMapKey(const SharedDriverState::AxisKey &key)
{
    return key.device + "|" + axisReadinessProtocolName(key.protocol) + "|" +
           std::to_string(key.motorId);
}

std::vector<MotorActionExecutor::Target> filterTargetsByDevice(
    const std::vector<MotorActionExecutor::Target> &targets,
    const std::string &device)
{
    std::vector<MotorActionExecutor::Target> filtered;
    for (const auto &target : targets) {
        if (target.canDevice == device) {
            filtered.push_back(target);
        }
    }
    return filtered;
}

} // namespace

LifecycleDriverOps::LifecycleDriverOps(std::shared_ptr<IDeviceManager> deviceManager,
                                       const MotorActionExecutor *motorActionExecutor)
    : deviceManager_(std::move(deviceManager)),
      motorActionExecutor_(motorActionExecutor)
{
}

void LifecycleDriverOps::configure(std::shared_ptr<IDeviceManager> deviceManager,
                                   const MotorActionExecutor *motorActionExecutor)
{
    {
        std::lock_guard<std::mutex> lock(targetsMutex_);
        deviceManager_ = std::move(deviceManager);
        motorActionExecutor_ = motorActionExecutor;
        activeTargets_.clear();
    }
    std::lock_guard<std::mutex> readinessLock(axisReadinessMutex_);
    axisRecoverTrackers_.clear();
}

void LifecycleDriverOps::setTargets(std::vector<MotorActionExecutor::Target> targets)
{
    {
        std::lock_guard<std::mutex> lock(targetsMutex_);
        targets_ = std::move(targets);
        activeTargets_.clear();
    }
    std::lock_guard<std::mutex> readinessLock(axisReadinessMutex_);
    axisRecoverTrackers_.clear();
}

void LifecycleDriverOps::setFeedbackFreshnessTimeoutNs(std::int64_t timeoutNs)
{
    std::lock_guard<std::mutex> readinessLock(axisReadinessMutex_);
    auto config = axisReadinessEvaluator_.config();
    config.feedbackFreshnessTimeoutNs = timeoutNs;
    axisReadinessEvaluator_.setConfig(config);
    axisRecoverTrackers_.clear();
}

std::vector<MotorActionExecutor::Target> LifecycleDriverOps::targetsSnapshot() const
{
    std::lock_guard<std::mutex> lock(targetsMutex_);
    return targets_;
}

std::vector<MotorActionExecutor::Target> LifecycleDriverOps::lifecycleTargetsSnapshot() const
{
    std::lock_guard<std::mutex> lock(targetsMutex_);
    return activeTargets_.empty() ? targets_ : activeTargets_;
}

void LifecycleDriverOps::activateTargetsForDevice(
    const std::string &device,
    const std::vector<MotorActionExecutor::Target> &deviceTargets) const
{
    std::lock_guard<std::mutex> lock(targetsMutex_);
    activeTargets_.erase(
        std::remove_if(activeTargets_.begin(),
                       activeTargets_.end(),
                       [&device](const MotorActionExecutor::Target &target) {
                           return target.canDevice == device;
                       }),
        activeTargets_.end());
    activeTargets_.insert(activeTargets_.end(), deviceTargets.begin(), deviceTargets.end());
}

void LifecycleDriverOps::deactivateTargetsForDevice(const std::string &device) const
{
    {
        std::lock_guard<std::mutex> lock(targetsMutex_);
        activeTargets_.erase(
            std::remove_if(activeTargets_.begin(),
                           activeTargets_.end(),
                           [&device](const MotorActionExecutor::Target &target) {
                               return target.canDevice == device;
                           }),
            activeTargets_.end());
    }
    std::lock_guard<std::mutex> readinessLock(axisReadinessMutex_);
    axisRecoverTrackers_.clear();
}

void LifecycleDriverOps::clearActiveTargets() const
{
    {
        std::lock_guard<std::mutex> lock(targetsMutex_);
        activeTargets_.clear();
    }
    std::lock_guard<std::mutex> readinessLock(axisReadinessMutex_);
    axisRecoverTrackers_.clear();
}

LifecycleDriverOps::Result LifecycleDriverOps::makeMotorActionFailureResult(
    MotorActionExecutor::Status status,
    const char *rejectedMessage,
    const char *protocolUnavailableMessage) const
{
    if (status == MotorActionExecutor::Status::DeviceNotReady) {
        return {false, "CAN device not ready."};
    }
    if (status == MotorActionExecutor::Status::ProtocolUnavailable) {
        return {false, protocolUnavailableMessage ? protocolUnavailableMessage : "Protocol not available."};
    }
    if (status == MotorActionExecutor::Status::Rejected) {
        return {false, rejectedMessage ? rejectedMessage : "Command rejected."};
    }
    return {false, "Command execution failed."};
}

LifecycleDriverOps::Result LifecycleDriverOps::runMotorBatchAction(
    const std::vector<MotorActionExecutor::Target> &targets,
    const MotorActionExecutor::Action &action,
    const char *operationName,
    const char *rejectedMessage,
    const char *protocolUnavailableMessage,
    bool requireAnyTarget) const
{
    if (!motorActionExecutor_) {
        return {false, "Motor action executor unavailable."};
    }
    if (targets.empty()) {
        return requireAnyTarget
                   ? Result{false, "No joints available."}
                   : Result{true, "No joints to process."};
    }

    const auto batch = motorActionExecutor_->executeBatch(targets, action, operationName);
    if (!batch.anySuccess && batch.anyFailure) {
        return makeMotorActionFailureResult(batch.firstFailure,
                                            rejectedMessage,
                                            protocolUnavailableMessage);
    }
    if (!batch.anySuccess && requireAnyTarget) {
        return {false, "No joints available."};
    }
    if (batch.anyFailure) {
        return makeMotorActionFailureResult(batch.firstFailure,
                                            rejectedMessage,
                                            protocolUnavailableMessage);
    }
    return {true, ""};
}

std::shared_ptr<CanProtocol> LifecycleDriverOps::getProtocol(const std::string &device,
                                                             CanType type) const
{
    if (!deviceManager_) {
        return nullptr;
    }
    return deviceManager_->getProtocol(device, type);
}

std::shared_ptr<std::mutex> LifecycleDriverOps::getDeviceMutex(const std::string &device) const
{
    if (!deviceManager_) {
        return nullptr;
    }
    return deviceManager_->getDeviceMutex(device);
}

bool LifecycleDriverOps::isDeviceReady(const std::string &device) const
{
    return deviceManager_ && deviceManager_->isDeviceReady(device);
}

std::shared_ptr<SharedDriverState> LifecycleDriverOps::getSharedDriverState() const
{
    if (!deviceManager_) {
        return nullptr;
    }
    return deviceManager_->getSharedDriverState();
}

AxisReadiness LifecycleDriverOps::evaluateAxisReadiness(
    const SharedDriverState::AxisKey &axisKey,
    const SharedDriverState::AxisFeedbackState &feedback,
    const SharedDriverState::AxisCommandState *command,
    AxisIntent intent,
    const SharedDriverState::DeviceHealthState *deviceHealth) const
{
    std::lock_guard<std::mutex> readinessLock(axisReadinessMutex_);
    AxisReadiness readiness =
        axisReadinessEvaluator_.Evaluate(feedback, command, intent, deviceHealth);

    const auto mapKey = axisReadinessMapKey(axisKey);
    if (intent != AxisIntent::Recover) {
        axisRecoverTrackers_.erase(mapKey);
        readiness.recoverConfirmed = readiness.axisReadyForEnable;
        return readiness;
    }

    if (!readiness.axisReadyForEnable) {
        axisRecoverTrackers_.erase(mapKey);
        readiness.recoverConfirmed = false;
        return readiness;
    }

    auto &tracker = axisRecoverTrackers_[mapKey];
    if (feedback.lastRxSteadyNs > 0 && feedback.lastRxSteadyNs != tracker.lastSampleNs) {
        tracker.lastSampleNs = feedback.lastRxSteadyNs;
        tracker.healthyCycles = std::min<std::uint32_t>(
            static_cast<std::uint32_t>(tracker.healthyCycles + 1),
            axisReadinessEvaluator_.recoverConfirmCycles());
    } else {
        tracker.healthyCycles = std::max<std::uint32_t>(tracker.healthyCycles, 1u);
    }

    readiness.recoverConfirmed =
        tracker.healthyCycles >= axisReadinessEvaluator_.recoverConfirmCycles();
    return readiness;
}

bool LifecycleDriverOps::queryMotorFault(const MotorActionExecutor::Target &target,
                                         bool *hasFault) const
{
    if (!hasFault) {
        return false;
    }

    if (const auto sharedState = getSharedDriverState()) {
        SharedDriverState::AxisFeedbackState feedback;
        if (sharedState->getAxisFeedback(
                MakeAxisKey(target.canDevice, target.protocol, target.motorId),
                &feedback) &&
            feedback.feedbackSeen && feedback.faultValid) {
            *hasFault = feedback.fault;
            return true;
        }
    }

    auto proto = getProtocol(target.canDevice, target.protocol);
    auto devMutex = getDeviceMutex(target.canDevice);
    if (!proto || !devMutex) {
        return false;
    }

    std::lock_guard<std::mutex> devLock(*devMutex);
    *hasFault = proto->hasFault(target.motorId);
    return true;
}

bool LifecycleDriverOps::queryMotorEnabled(const MotorActionExecutor::Target &target,
                                           bool *enabled) const
{
    if (!enabled) {
        return false;
    }

    if (const auto sharedState = getSharedDriverState()) {
        SharedDriverState::AxisFeedbackState feedback;
        if (sharedState->getAxisFeedback(
                MakeAxisKey(target.canDevice, target.protocol, target.motorId),
                &feedback) &&
            feedback.feedbackSeen && feedback.enabledValid) {
            *enabled = feedback.enabled;
            return true;
        }
    }

    auto proto = getProtocol(target.canDevice, target.protocol);
    auto devMutex = getDeviceMutex(target.canDevice);
    if (!proto || !devMutex) {
        return false;
    }

    std::lock_guard<std::mutex> devLock(*devMutex);
    *enabled = proto->isEnabled(target.motorId);
    return true;
}

LifecycleDriverOps::Result LifecycleDriverOps::initializeDevice(const std::string &device,
                                                                bool loopback) const
{
    const auto prepare = prepareDevice(device, loopback);
    if (!prepare.ok) {
        return prepare;
    }
    activateTargetsForDevice(device, filterTargetsByDevice(targetsSnapshot(), device));
    return {true, "initialized (standby)"};
}

LifecycleDriverOps::Result LifecycleDriverOps::shutdownDevice(const std::string &device) const
{
    if (!deviceManager_) {
        return {false, "Device manager unavailable."};
    }

    deviceManager_->shutdownDevice(device);
    deactivateTargetsForDevice(device);
    return {true, ""};
}

LifecycleDriverOps::Result LifecycleDriverOps::prepareDevice(const std::string &device,
                                                             bool loopback) const
{
    if (!deviceManager_) {
        return {false, "Device manager unavailable."};
    }

    const auto targets = targetsSnapshot();
    const auto deviceTargets = filterTargetsByDevice(targets, device);
    if (deviceTargets.empty()) {
        return {false, "No joints available for device " + device};
    }

    std::vector<std::pair<CanType, MotorID>> motors;
    for (const auto &target : deviceTargets) {
        motors.emplace_back(target.protocol, target.motorId);
    }

    if (!deviceManager_->initDevice(device, motors, loopback)) {
        return {false, "Failed to initialize " + device};
    }
    return {true, ""};
}

LifecycleDriverOps::Result LifecycleDriverOps::enableDevice(const std::string &device) const
{
    if (!motorActionExecutor_) {
        return {false, "Motor action executor unavailable."};
    }

    const auto targets = targetsSnapshot();
    const auto deviceTargets = filterTargetsByDevice(targets, device);
    if (deviceTargets.empty()) {
        return {false, "No joints available for device " + device};
    }

    const auto batch = motorActionExecutor_->executeBatch(
        deviceTargets,
        [](const std::shared_ptr<CanProtocol> &proto, MotorID id) {
            return proto->Enable(id);
        },
        "Init enable");
    if (batch.anyFailure) {
        if (!batch.succeededTargets.empty()) {
            const auto rollback = motorActionExecutor_->executeBatch(
                batch.succeededTargets,
                [](const std::shared_ptr<CanProtocol> &proto, MotorID id) {
                    return proto->Disable(id);
                },
                "Init enable rollback");
            if (rollback.anyFailure) {
                return makeMotorActionFailureResult(
                    rollback.firstFailure,
                    "Init enable rollback failed after partial success.",
                    "Protocol not available during init enable rollback.");
            }
        }
        return makeMotorActionFailureResult(batch.firstFailure,
                                            "Enable command rejected.",
                                            "Protocol not available.");
    }

    const auto confirmed = waitForEnabledTargets(deviceTargets,
                                                 std::chrono::seconds(2),
                                                 std::chrono::milliseconds(50));
    if (!confirmed.ok) {
        const auto rollback = motorActionExecutor_->executeBatch(
            deviceTargets,
            [](const std::shared_ptr<CanProtocol> &proto, MotorID id) {
                return proto->Disable(id);
            },
            "Init enable confirmation rollback");
        if (rollback.anyFailure) {
            return makeMotorActionFailureResult(
                rollback.firstFailure,
                "Init enable confirmation rollback failed after enable timeout.",
                "Protocol not available during init enable confirmation rollback.");
        }
        return confirmed;
    }

    return {true, "enabled (armed)"};
}

LifecycleDriverOps::Result LifecycleDriverOps::enableAll() const
{
    if (!motorActionExecutor_) {
        return {false, "Motor action executor unavailable."};
    }

    const auto targets = lifecycleTargetsSnapshot();
    if (targets.empty()) {
        return {false, "No joints available for enable."};
    }

    const auto batch = motorActionExecutor_->executeBatch(
        targets,
        [](const std::shared_ptr<CanProtocol> &proto, MotorID id) {
            return proto->Enable(id);
        },
        "Enable");
    if (batch.anyFailure) {
        if (!batch.succeededTargets.empty()) {
            const auto rollback = motorActionExecutor_->executeBatch(
                batch.succeededTargets,
                [](const std::shared_ptr<CanProtocol> &proto, MotorID id) {
                    return proto->Disable(id);
                },
                "Enable rollback");
            if (rollback.anyFailure) {
                return makeMotorActionFailureResult(
                    rollback.firstFailure,
                    "Enable rollback failed after partial success.",
                    "Protocol not available during enable rollback.");
            }
        }
        return makeMotorActionFailureResult(batch.firstFailure,
                                            "Enable command rejected.",
                                            "Protocol not available.");
    }
    if (!batch.anySuccess) {
        return {false, "No joints available for enable."};
    }

    const auto confirmed = waitForEnabledTargets(targets,
                                                 std::chrono::seconds(2),
                                                 std::chrono::milliseconds(50));
    if (!confirmed.ok) {
        if (!batch.succeededTargets.empty()) {
            const auto rollback = motorActionExecutor_->executeBatch(
                batch.succeededTargets,
                [](const std::shared_ptr<CanProtocol> &proto, MotorID id) {
                    return proto->Disable(id);
                },
                "Enable confirmation rollback");
            if (rollback.anyFailure) {
                return makeMotorActionFailureResult(
                    rollback.firstFailure,
                    "Enable confirmation rollback failed after enable timeout.",
                    "Protocol not available during enable confirmation rollback.");
            }
        }
        return confirmed;
    }

    return {true, "enabled (armed)"};
}

LifecycleDriverOps::Result LifecycleDriverOps::disableAll() const
{
    return runMotorBatchAction(
        lifecycleTargetsSnapshot(),
        [](const std::shared_ptr<CanProtocol> &proto, MotorID id) {
            return proto->Disable(id);
        },
        "Disable",
        "Disable command rejected.",
        "Protocol not available.",
        false);
}

LifecycleDriverOps::Result LifecycleDriverOps::haltAll() const
{
    return runMotorBatchAction(
        lifecycleTargetsSnapshot(),
        [](const std::shared_ptr<CanProtocol> &proto, MotorID id) {
            return proto->Stop(id);
        },
        "Halt",
        "Halt command rejected.",
        "Protocol not available.",
        false);
}

LifecycleDriverOps::Result LifecycleDriverOps::recoverAll() const
{
    if (!motorActionExecutor_) {
        return {false, "Motor action executor unavailable."};
    }

    const auto targets = lifecycleTargetsSnapshot();
    if (targets.empty()) {
        return {false, "No motors available for recover."};
    }

    const auto batch = motorActionExecutor_->executeBatch(
        targets,
        [](const std::shared_ptr<CanProtocol> &proto, MotorID id) {
            return proto->ResetFault(id);
        },
        "Recover");
    if (!batch.anySuccess && batch.anyFailure) {
        return makeMotorActionFailureResult(batch.firstFailure,
                                            "Recover command rejected.",
                                            "Protocol fault-reset path not available.");
    }
    if (!batch.anySuccess) {
        return {false, "No motors available for recover."};
    }

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (std::chrono::steady_clock::now() < deadline) {
        bool allHealthy = true;
        for (const auto &target : targets) {
            if (!isDeviceReady(target.canDevice)) {
                allHealthy = false;
                break;
            }
            if (const auto sharedState = getSharedDriverState()) {
                const auto axisKey =
                    MakeAxisKey(target.canDevice, target.protocol, target.motorId);
                SharedDriverState::AxisFeedbackState feedback;
                if (sharedState->getAxisFeedback(axisKey, &feedback)) {
                    SharedDriverState::AxisCommandState command;
                    const SharedDriverState::AxisCommandState *commandPtr =
                        sharedState->getAxisCommand(axisKey, &command) ? &command : nullptr;
                    SharedDriverState::DeviceHealthState deviceHealth;
                    const SharedDriverState::DeviceHealthState *deviceHealthPtr =
                        sharedState->getDeviceHealth(target.canDevice, &deviceHealth)
                            ? &deviceHealth
                            : nullptr;
                    if (!feedback.faultValid) {
                        bool hasFault = false;
                        if (queryMotorFault(target, &hasFault)) {
                            feedback.fault = hasFault;
                            feedback.faultValid = true;
                        }
                    }

                    const auto readiness = evaluateAxisReadiness(axisKey,
                                                                 feedback,
                                                                 commandPtr,
                                                                 AxisIntent::Recover,
                                                                 deviceHealthPtr);
                    if (!AxisReadinessEvaluator::RecoverConfirmed(readiness)) {
                        allHealthy = false;
                        break;
                    }
                    continue;
                }
            }

            bool hasFault = false;
            if (!queryMotorFault(target, &hasFault)) {
                return {false, "Protocol not available during fault verification."};
            }
            if (hasFault) {
                allHealthy = false;
                break;
            }
        }
        if (allHealthy) {
            return {true, ""};
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    return {false, "Recover timeout: fault still active."};
}

LifecycleDriverOps::Result LifecycleDriverOps::shutdownAll(bool force) const
{
    (void)force;
    if (!deviceManager_) {
        return {false, "Device manager unavailable."};
    }

    deviceManager_->shutdownAll();
    clearActiveTargets();
    return {true, ""};
}

LifecycleDriverOps::Result LifecycleDriverOps::waitForEnabledTargets(
    const std::vector<MotorActionExecutor::Target> &targets,
    std::chrono::milliseconds timeout,
    std::chrono::milliseconds pollInterval) const
{
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    std::string firstFailureDetail;

    while (std::chrono::steady_clock::now() < deadline) {
        bool allEnabled = true;
        firstFailureDetail.clear();

        for (const auto &target : targets) {
            if (!isDeviceReady(target.canDevice)) {
                allEnabled = false;
                if (firstFailureDetail.empty()) {
                    firstFailureDetail = "CAN device not ready.";
                }
                break;
            }

            if (const auto sharedState = getSharedDriverState()) {
                const auto axisKey = MakeAxisKey(target.canDevice, target.protocol, target.motorId);
                SharedDriverState::AxisFeedbackState feedback;
                if (sharedState->getAxisFeedback(axisKey, &feedback)) {
                    SharedDriverState::AxisCommandState command;
                    const SharedDriverState::AxisCommandState *commandPtr =
                        sharedState->getAxisCommand(axisKey, &command) ? &command : nullptr;

                    SharedDriverState::DeviceHealthState deviceHealth;
                    const SharedDriverState::DeviceHealthState *deviceHealthPtr =
                        sharedState->getDeviceHealth(target.canDevice, &deviceHealth)
                            ? &deviceHealth
                            : nullptr;
                    if (!feedback.faultValid) {
                        bool hasFault = false;
                        if (queryMotorFault(target, &hasFault)) {
                            feedback.fault = hasFault;
                            feedback.faultValid = true;
                        }
                    }
                    if (!feedback.enabledValid || !feedback.enabled) {
                        bool enabled = false;
                        if (queryMotorEnabled(target, &enabled)) {
                            feedback.enabled = enabled;
                            feedback.enabledValid = true;
                        }
                    }

                    const auto readiness = evaluateAxisReadiness(
                        axisKey,
                        feedback,
                        commandPtr,
                        AxisIntent::Enable,
                        deviceHealthPtr);
                    if (!(AxisReadinessEvaluator::ReadyForEnable(readiness) &&
                          readiness.enabledReady)) {
                        allEnabled = false;
                        if (firstFailureDetail.empty()) {
                            firstFailureDetail = target.name + ": " +
                                                 AxisReadinessEvaluator::DescribeBlockReason(readiness);
                        }
                        break;
                    }
                    continue;
                }
            }

            bool enabled = false;
            if (!queryMotorEnabled(target, &enabled)) {
                allEnabled = false;
                if (firstFailureDetail.empty()) {
                    firstFailureDetail = target.name + ": Protocol not available.";
                }
                break;
            }
            if (!enabled) {
                allEnabled = false;
                if (firstFailureDetail.empty()) {
                    firstFailureDetail = target.name + ": Axis not enabled.";
                }
                break;
            }
        }

        if (allEnabled) {
            return {true, "enabled (armed)"};
        }

        std::this_thread::sleep_for(pollInterval);
    }

    if (firstFailureDetail.empty()) {
        firstFailureDetail = "Axis not enabled.";
    }
    return {false, "Enable confirmation timeout: " + firstFailureDetail};
}

bool LifecycleDriverOps::anyFaultActive() const
{
    for (const auto &target : lifecycleTargetsSnapshot()) {
        bool hasFault = false;
        if (queryMotorFault(target, &hasFault) && hasFault) {
            return true;
        }
    }
    return false;
}

bool LifecycleDriverOps::enableHealthy(std::string *detail) const
{
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(750);
    std::string lastDetail;
    while (true) {
        lastDetail.clear();
        if (enableHealthyOnce(&lastDetail)) {
            if (detail) {
                detail->clear();
            }
            return true;
        }

        const bool retryable = lastDetail.find("Feedback degraded.") != std::string::npos ||
                               lastDetail.find("Feedback offline.") != std::string::npos ||
                               lastDetail.find("Feedback stale.") != std::string::npos;
        if (!retryable || std::chrono::steady_clock::now() >= deadline) {
            if (detail) {
                *detail = lastDetail;
            }
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

bool LifecycleDriverOps::enableHealthySnapshot(std::string *detail) const
{
    return enableHealthyOnce(detail);
}

bool LifecycleDriverOps::enableHealthyOnce(std::string *detail) const
{
    for (const auto &target : lifecycleTargetsSnapshot()) {
        if (!isDeviceReady(target.canDevice)) {
            if (detail) {
                *detail = "CAN device not ready.";
            }
            return false;
        }
        if (const auto sharedState = getSharedDriverState()) {
            const auto axisKey = MakeAxisKey(target.canDevice, target.protocol, target.motorId);
            SharedDriverState::AxisFeedbackState feedback;
            if (sharedState->getAxisFeedback(axisKey, &feedback)) {
                SharedDriverState::AxisCommandState command;
                const SharedDriverState::AxisCommandState *commandPtr =
                    sharedState->getAxisCommand(axisKey, &command) ? &command : nullptr;

                SharedDriverState::DeviceHealthState deviceHealth;
                const SharedDriverState::DeviceHealthState *deviceHealthPtr =
                    sharedState->getDeviceHealth(target.canDevice, &deviceHealth)
                        ? &deviceHealth
                        : nullptr;
                if (!feedback.faultValid) {
                    bool hasFault = false;
                    if (queryMotorFault(target, &hasFault)) {
                        feedback.fault = hasFault;
                        feedback.faultValid = true;
                    }
                }

                const auto readiness = evaluateAxisReadiness(
                    axisKey,
                    feedback,
                    commandPtr,
                    AxisIntent::Enable,
                    deviceHealthPtr);
                if (!AxisReadinessEvaluator::ReadyForEnable(readiness)) {
                    if (detail) {
                        *detail = target.name + ": " +
                                  AxisReadinessEvaluator::DescribeBlockReason(readiness);
                    }
                    return false;
                }
                continue;
            }
        }
        bool hasFault = false;
        if (!queryMotorFault(target, &hasFault)) {
            if (detail) {
                *detail = "Protocol not available.";
            }
            return false;
        }
        if (hasFault) {
            if (detail) {
                *detail = "Fault still active.";
            }
            return false;
        }
    }
    return true;
}

bool LifecycleDriverOps::motionHealthy(std::string *detail) const
{
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(1250);
    std::string lastDetail;
    while (true) {
        lastDetail.clear();
        if (motionHealthyOnce(&lastDetail)) {
            if (detail) {
                detail->clear();
            }
            return true;
        }

        const bool retryable = lastDetail.find("Feedback degraded.") != std::string::npos ||
                               lastDetail.find("Feedback offline.") != std::string::npos ||
                               lastDetail.find("Feedback stale.") != std::string::npos;
        if (!retryable || std::chrono::steady_clock::now() >= deadline) {
            if (detail) {
                *detail = lastDetail;
            }
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

bool LifecycleDriverOps::motionHealthySnapshot(std::string *detail) const
{
    return motionHealthyOnce(detail);
}

bool LifecycleDriverOps::motionHealthyOnce(std::string *detail) const
{
    for (const auto &target : lifecycleTargetsSnapshot()) {
        if (!isDeviceReady(target.canDevice)) {
            if (detail) {
                *detail = "CAN device not ready.";
            }
            return false;
        }
        if (const auto sharedState = getSharedDriverState()) {
            const auto axisKey = MakeAxisKey(target.canDevice, target.protocol, target.motorId);
            SharedDriverState::AxisFeedbackState feedback;
            if (sharedState->getAxisFeedback(axisKey, &feedback)) {
                SharedDriverState::AxisCommandState command;
                const SharedDriverState::AxisCommandState *commandPtr =
                    sharedState->getAxisCommand(axisKey, &command) ? &command : nullptr;

                SharedDriverState::DeviceHealthState deviceHealth;
                const SharedDriverState::DeviceHealthState *deviceHealthPtr =
                    sharedState->getDeviceHealth(target.canDevice, &deviceHealth)
                        ? &deviceHealth
                        : nullptr;
                if (!feedback.faultValid) {
                    bool hasFault = false;
                    if (queryMotorFault(target, &hasFault)) {
                        feedback.fault = hasFault;
                        feedback.faultValid = true;
                    }
                }
                if (!feedback.enabledValid) {
                    bool enabled = false;
                    if (queryMotorEnabled(target, &enabled)) {
                        feedback.enabled = enabled;
                        feedback.enabledValid = true;
                    }
                }
                if ((target.protocol == CanType::MT || target.protocol == CanType::PP ||
                     target.protocol == CanType::ECB) &&
                    !feedback.modeValid &&
                    commandPtr != nullptr &&
                    commandPtr->desiredModeValid) {
                    feedback.mode = commandPtr->desiredMode;
                    feedback.modeValid = true;
                }

                const auto readiness = evaluateAxisReadiness(
                    axisKey,
                    feedback,
                    commandPtr,
                    sharedState->getAxisIntent(axisKey),
                    deviceHealthPtr);
                if (!AxisReadinessEvaluator::ReadyForRun(readiness)) {
                    if (detail) {
                        *detail = target.name + ": " +
                                  AxisReadinessEvaluator::DescribeBlockReason(readiness);
                    }
                    return false;
                }
                continue;
            }
        }
        bool hasFault = false;
        if (!queryMotorFault(target, &hasFault)) {
            if (detail) {
                *detail = "Protocol not available.";
            }
            return false;
        }
        if (hasFault) {
            if (detail) {
                *detail = "Fault still active.";
            }
            return false;
        }
    }
    return true;
}

} // namespace can_driver
