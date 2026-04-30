#include "can_driver/DeviceManager.h"
#include "can_driver/DeviceRuntime.h"
#include "can_driver/RefreshScheduler.h"

#include "can_driver/MotorID.h"

#include <algorithm>
#include <chrono>
#include <cmath>

#include <ros/ros.h>

namespace {

constexpr std::uint64_t kQueryPressureHoldCycles = 20;
constexpr const char *kUdpPrefix = "udp://";
constexpr const char *kEcbPrefix = "ecb://";

std::chrono::milliseconds clampRefreshSleep(std::chrono::milliseconds sleepFor)
{
    if (sleepFor < std::chrono::milliseconds(1)) {
        return std::chrono::milliseconds(1);
    }
    return sleepFor;
}

std::int64_t sharedAxisLastRxNs(const std::shared_ptr<can_driver::SharedDriverState> &sharedState,
                                const std::string &deviceName,
                                CanType protocol,
                                MotorID motorId)
{
    if (!sharedState) {
        return 0;
    }

    can_driver::SharedDriverState::AxisFeedbackState feedback;
    if (!sharedState->getAxisFeedback(
            can_driver::MakeAxisKey(deviceName, protocol, motorId),
            &feedback)) {
        return 0;
    }
    return feedback.lastRxSteadyNs;
}

std::vector<std::uint8_t> normalizeMotorIds(const std::vector<MotorID> &motorIds,
                                            CanType protocol)
{
    std::vector<std::uint8_t> normalized;
    normalized.reserve(motorIds.size());
    for (const auto motorId : motorIds) {
        if (protocol == CanType::MT) {
            normalized.push_back(can_driver::toMtProtocolNodeId(motorId));
        } else {
            normalized.push_back(can_driver::toProtocolNodeId(motorId));
        }
    }
    return normalized;
}

std::vector<MotorID> normalizeMtSystemMotorIds(const std::vector<MotorID> &motorIds)
{
    std::vector<MotorID> normalized;
    normalized.reserve(motorIds.size());
    for (const auto motorId : motorIds) {
        normalized.push_back(can_driver::motorIdFromMtProtocolNodeId(
            can_driver::toMtProtocolNodeId(motorId)));
    }
    return normalized;
}

can_driver::PpAxisRefreshSnapshot buildPpAxisRefreshSnapshot(
    const std::shared_ptr<can_driver::SharedDriverState> &sharedState,
    const std::string &deviceName,
    std::uint8_t motorId)
{
    can_driver::PpAxisRefreshSnapshot snapshot;
    if (!sharedState) {
        return snapshot;
    }

    const auto axisKey =
        can_driver::MakeAxisKey(deviceName, CanType::PP,
                                can_driver::motorIdFromProtocolNodeId(motorId));

    can_driver::SharedDriverState::AxisFeedbackState feedback;
    if (sharedState->getAxisFeedback(axisKey, &feedback)) {
        snapshot.feedbackSeen = feedback.feedbackSeen;
        if (feedback.enabledValid) {
            snapshot.enabled = feedback.enabled;
        }
        if (feedback.faultValid) {
            snapshot.fault = feedback.fault;
        }
        snapshot.degraded = feedback.degraded;
        if (feedback.modeValid) {
            snapshot.feedbackMode = feedback.mode;
        }
    }

    can_driver::SharedDriverState::AxisCommandState command;
    if (sharedState->getAxisCommand(axisKey, &command)) {
        snapshot.desiredModeValid = command.desiredModeValid;
        snapshot.desiredMode = command.desiredMode;
    }

    snapshot.intent = sharedState->getAxisIntent(axisKey);
    return snapshot;
}

can_driver::DmAxisRefreshSnapshot buildDmAxisRefreshSnapshot(
    const std::shared_ptr<can_driver::SharedDriverState> &sharedState,
    const std::string &deviceName,
    std::uint8_t motorId)
{
    can_driver::DmAxisRefreshSnapshot snapshot;
    if (!sharedState) {
        return snapshot;
    }

    const auto axisKey =
        can_driver::MakeAxisKey(deviceName, CanType::DM,
                                can_driver::motorIdFromProtocolNodeId(motorId));

    can_driver::SharedDriverState::AxisFeedbackState feedback;
    if (sharedState->getAxisFeedback(axisKey, &feedback)) {
        snapshot.feedbackSeen = feedback.feedbackSeen;
        if (feedback.enabledValid) {
            snapshot.enabled = feedback.enabled;
        }
        if (feedback.faultValid) {
            snapshot.fault = feedback.fault;
        }
    }

    snapshot.intent = sharedState->getAxisIntent(axisKey);
    return snapshot;
}

} // namespace

void DeviceManager::setRefreshInterFrameGapUs(uint32_t gapUs)
{
    std::unique_lock<std::shared_mutex> lock(mutex_);
    refreshInterFrameGap_ = std::chrono::microseconds(gapUs);
    for (auto &entry : deviceRefreshRuntimes_) {
        if (!entry.second) {
            continue;
        }
        {
            std::lock_guard<std::mutex> scheduleLock(entry.second->scheduleMutex);
            entry.second->interFrameGap = refreshInterFrameGap_;
            entry.second->nextEligibleIssueTime = std::chrono::steady_clock::time_point {};
        }
        if (entry.second->worker) {
            entry.second->worker->notify();
        }
    }
}

bool DeviceManager::isUdpDevice(const std::string &device) const
{
    return device.rfind(kUdpPrefix, 0) == 0;
}

bool DeviceManager::isEcbDevice(const std::string &device) const
{
    return device.rfind(kEcbPrefix, 0) == 0;
}

std::shared_ptr<CanTransport> DeviceManager::getTransportBaseLocked(const std::string &device) const
{
    const auto canIt = transports_.find(device);
    if (canIt != transports_.end()) {
        return std::static_pointer_cast<CanTransport>(canIt->second);
    }
    const auto udpIt = udpTransports_.find(device);
    if (udpIt != udpTransports_.end()) {
        return std::static_pointer_cast<CanTransport>(udpIt->second);
    }
    return nullptr;
}

bool DeviceManager::shutdownTransportLocked(const std::string &device)
{
    const auto canIt = transports_.find(device);
    if (canIt != transports_.end() && canIt->second) {
        canIt->second->shutdown();
        return true;
    }
    const auto udpIt = udpTransports_.find(device);
    if (udpIt != udpTransports_.end() && udpIt->second) {
        udpIt->second->shutdown();
        return true;
    }
    return false;
}

bool DeviceManager::initializeTransportLocked(const std::string &device, bool loopback)
{
    if (isUdpDevice(device)) {
        const auto it = udpTransports_.find(device);
        if (it == udpTransports_.end()) {
            auto transport = std::make_shared<UdpCanTransport>();
            if (!transport->initialize(device)) {
                ROS_ERROR("[CanDriverHW] Failed to initialize UDP device '%s'.", device.c_str());
                return false;
            }
            udpTransports_[device] = transport;
        } else if (!it->second->initialize(device)) {
            ROS_ERROR("[CanDriverHW] Failed to re-initialize UDP device '%s'.", device.c_str());
            return false;
        }
        return true;
    }

    const auto it = transports_.find(device);
    if (it == transports_.end()) {
        auto transport = std::make_shared<SocketCanController>();
        if (!transport->initialize(device, loopback)) {
            ROS_ERROR("[CanDriverHW] Failed to initialize CAN device '%s'.", device.c_str());
            return false;
        }
        transports_[device] = transport;
    } else if (!it->second->initialize(device, loopback)) {
        ROS_ERROR("[CanDriverHW] Failed to re-initialize CAN device '%s'.", device.c_str());
        return false;
    }

    return true;
}

void DeviceManager::syncDeviceRefreshRuntimeLocked(const std::string &device)
{
    auto &runtime = deviceRefreshRuntimes_[device];
    if (!runtime) {
        runtime = std::make_shared<DeviceRefreshRuntime>();
    }
    runtime->deviceName = device;
    runtime->sharedState = sharedState_;
    const auto mtIt = mtProtocols_.find(device);
    runtime->mtProtocol =
        (mtIt != mtProtocols_.end()) ? mtIt->second : std::shared_ptr<MtCan>();
    const auto ppIt = eyouProtocols_.find(device);
    runtime->ppProtocol = (ppIt != eyouProtocols_.end()) ? ppIt->second
                                                         : std::shared_ptr<EyouCan>();
    const auto dmIt = damiaoProtocols_.find(device);
    runtime->dmProtocol = (dmIt != damiaoProtocols_.end()) ? dmIt->second
                                                           : std::shared_ptr<DamiaoCan>();
    runtime->interFrameGap = refreshInterFrameGap_;

    if (runtime->worker) {
        return;
    }

    std::weak_ptr<DeviceRefreshRuntime> weakRuntime = runtime;
    runtime->worker = std::make_shared<can_driver::DeviceRefreshWorker>(
        [weakRuntime]() {
            const auto runtime = weakRuntime.lock();
            if (!runtime) {
                return;
            }

            bool queryPressureActive = false;
            const auto now = std::chrono::steady_clock::now();
            const auto sharedState = runtime->sharedState.lock();
            {
                std::lock_guard<std::mutex> lock(runtime->scheduleMutex);
                if (runtime->pendingRefresh.active) {
                    const auto lastRx = sharedAxisLastRxNs(sharedState,
                                                           runtime->deviceName,
                                                           runtime->pendingRefresh.protocol,
                                                           runtime->pendingRefresh.motorId);
                    if (lastRx > runtime->pendingRefresh.observedLastRxSteadyNs ||
                        now >= runtime->pendingRefresh.deadline) {
                        runtime->pendingRefresh.active = false;
                    }
                }
                if (runtime->nextEligibleIssueTime != std::chrono::steady_clock::time_point {} &&
                    now < runtime->nextEligibleIssueTime) {
                    return;
                }
                if (runtime->pendingRefresh.active) {
                    return;
                }

                const auto cycle = runtime->refreshCycleCount++;
                if (sharedState) {
                    can_driver::SharedDriverState::DeviceHealthState deviceHealth;
                    if (sharedState->getDeviceHealth(runtime->deviceName, &deviceHealth)) {
                        if (deviceHealth.txBackpressure > runtime->lastObservedTxBackpressure) {
                            runtime->queryPressureUntilCycle = std::max(
                                runtime->queryPressureUntilCycle, cycle + kQueryPressureHoldCycles);
                        }
                        runtime->lastObservedTxBackpressure = deviceHealth.txBackpressure;
                    }
                }
                queryPressureActive = cycle < runtime->queryPressureUntilCycle;
            }

            enum class AttemptedProtocol : std::size_t { Mt = 0, Pp = 1, Dm = 2 };
            bool issuedAny = false;
            for (std::size_t protocolTry = 0; protocolTry < 3 && !issuedAny; ++protocolTry) {
                const auto selected = static_cast<AttemptedProtocol>(
                    (runtime->protocolRoundRobinCursor + protocolTry) % 3);

                if (selected == AttemptedProtocol::Mt &&
                    runtime->mtActive.load(std::memory_order_acquire)) {
                    const auto protocol = runtime->mtProtocol.lock();
                    std::vector<MotorID> motorIds;
                    std::size_t cursor = 0;
                    std::uint64_t cycle = 0;
                    bool due = false;
                    {
                        std::lock_guard<std::mutex> lock(runtime->scheduleMutex);
                        due = (runtime->nextMtTick == std::chrono::steady_clock::time_point {}) ||
                              (now >= runtime->nextMtTick);
                        if (due) {
                            cycle = runtime->mtScheduleCycleCount++;
                            motorIds = runtime->mtMotorIds;
                            cursor = runtime->mtMotorCursor;
                        }
                    }
                    if (protocol && due && !motorIds.empty()) {
                        for (std::size_t offset = 0; offset < motorIds.size() && !issuedAny; ++offset) {
                            const std::size_t index = (cursor + offset) % motorIds.size();
                            const auto motorId = motorIds[index];
                            const auto plan = can_driver::BuildMtRefreshPlan(cycle, index, queryPressureActive);
                            for (std::size_t j = 0; j < plan.count && !issuedAny; ++j) {
                                const bool issued = protocol->issueRefreshQuery(motorId, plan.items[j]);
                                if (!issued) {
                                    continue;
                                }
                                issuedAny = true;
                                std::lock_guard<std::mutex> lock(runtime->scheduleMutex);
                                runtime->mtMotorCursor = (index + 1) % motorIds.size();
                                runtime->nextMtTick = std::chrono::steady_clock::now() +
                                                      clampRefreshSleep(protocol->refreshSleepInterval());
                                runtime->nextEligibleIssueTime = std::chrono::steady_clock::now() +
                                                                 runtime->interFrameGap;
                                runtime->pendingRefresh.protocol = CanType::MT;
                                runtime->pendingRefresh.motorId = motorId;
                                runtime->pendingRefresh.observedLastRxSteadyNs =
                                    sharedAxisLastRxNs(sharedState, runtime->deviceName, CanType::MT, motorId);
                                runtime->pendingRefresh.deadline = std::chrono::steady_clock::now() +
                                                                   clampRefreshSleep(protocol->readResponseTimeout());
                                runtime->pendingRefresh.active = true;
                                runtime->protocolRoundRobinCursor = (static_cast<std::size_t>(AttemptedProtocol::Pp));
                            }
                        }
                    }
                }

                if (selected == AttemptedProtocol::Pp &&
                    runtime->ppActive.load(std::memory_order_acquire) && !issuedAny) {
                    const auto protocol = runtime->ppProtocol.lock();
                    std::vector<std::uint8_t> motorIds;
                    std::size_t cursor = 0;
                    bool due = false;
                    {
                        std::lock_guard<std::mutex> lock(runtime->scheduleMutex);
                        due = (runtime->nextPpTick == std::chrono::steady_clock::time_point {}) ||
                              (now >= runtime->nextPpTick);
                        if (due) {
                            ++runtime->ppScheduleCycleCount;
                            motorIds = runtime->ppMotorIds;
                            cursor = runtime->ppMotorCursor;
                        }
                    }
                    if (protocol && due && !motorIds.empty()) {
                        for (std::size_t offset = 0; offset < motorIds.size() && !issuedAny; ++offset) {
                            const std::size_t index = (cursor + offset) % motorIds.size();
                            const auto motorId = motorIds[index];
                            const auto systemMotorId = can_driver::motorIdFromProtocolNodeId(motorId);
                            const auto snapshot = buildPpAxisRefreshSnapshot(
                                sharedState, runtime->deviceName, motorId);
                            can_driver::PpRefreshPlan plan;
                            {
                                std::lock_guard<std::mutex> lock(runtime->scheduleMutex);
                                auto &scheduleState = runtime->ppScheduleStates[motorId];
                                plan = can_driver::BuildPpRefreshPlan(
                                    now, queryPressureActive, snapshot, &scheduleState);
                                for (std::size_t j = 0; j < plan.count; ++j) {
                                    can_driver::NotePpRefreshQueryDue(now, &scheduleState, plan.items[j]);
                                }
                            }
                            for (std::size_t j = 0; j < plan.count && !issuedAny; ++j) {
                                const bool issued = protocol->issueRefreshQuery(systemMotorId, plan.items[j]);
                                if (!issued) {
                                    continue;
                                }
                                issuedAny = true;
                                std::lock_guard<std::mutex> lock(runtime->scheduleMutex);
                                auto &scheduleState = runtime->ppScheduleStates[motorId];
                                can_driver::NotePpRefreshQueryIssued(now, &scheduleState, plan.items[j]);
                                runtime->ppMotorCursor = (index + 1) % motorIds.size();
                                runtime->nextPpTick = std::chrono::steady_clock::now() +
                                                      clampRefreshSleep(protocol->refreshSleepInterval());
                                runtime->nextEligibleIssueTime = std::chrono::steady_clock::now() +
                                                                 runtime->interFrameGap;
                                runtime->pendingRefresh.protocol = CanType::PP;
                                runtime->pendingRefresh.motorId = systemMotorId;
                                runtime->pendingRefresh.observedLastRxSteadyNs =
                                    sharedAxisLastRxNs(sharedState, runtime->deviceName, CanType::PP, systemMotorId);
                                runtime->pendingRefresh.deadline = std::chrono::steady_clock::now() +
                                                                   clampRefreshSleep(protocol->refreshSleepInterval());
                                runtime->pendingRefresh.active = true;
                                runtime->protocolRoundRobinCursor = (static_cast<std::size_t>(AttemptedProtocol::Dm));
                            }
                        }
                    }
                }

                if (selected == AttemptedProtocol::Dm &&
                    runtime->dmActive.load(std::memory_order_acquire) && !issuedAny) {
                    const auto protocol = runtime->dmProtocol.lock();
                    std::vector<std::uint8_t> motorIds;
                    std::size_t cursor = 0;
                    bool due = false;
                    {
                        std::lock_guard<std::mutex> lock(runtime->scheduleMutex);
                        due = (runtime->nextDmTick == std::chrono::steady_clock::time_point {}) ||
                              (now >= runtime->nextDmTick);
                        if (due) {
                            motorIds = runtime->dmMotorIds;
                            cursor = runtime->dmMotorCursor;
                        }
                    }
                    if (protocol && due && !motorIds.empty()) {
                        for (std::size_t offset = 0; offset < motorIds.size() && !issuedAny; ++offset) {
                            const std::size_t index = (cursor + offset) % motorIds.size();
                            const auto motorId = motorIds[index];
                            const auto systemMotorId = can_driver::motorIdFromProtocolNodeId(motorId);
                            can_driver::DmRefreshPlan plan;
                            {
                                std::lock_guard<std::mutex> lock(runtime->scheduleMutex);
                                auto &scheduleState = runtime->dmScheduleStates[motorId];
                                plan = can_driver::BuildDmRefreshPlan(
                                    now,
                                    buildDmAxisRefreshSnapshot(sharedState,
                                                               runtime->deviceName,
                                                               motorId),
                                    &scheduleState);
                            }
                            for (std::size_t i = 0; i < plan.count && !issuedAny; ++i) {
                                const bool issued = protocol->issueRefreshQuery(systemMotorId, plan.items[i]);
                                if (!issued) {
                                    continue;
                                }
                                issuedAny = true;
                                std::lock_guard<std::mutex> lock(runtime->scheduleMutex);
                                auto &scheduleState = runtime->dmScheduleStates[motorId];
                                can_driver::NoteDmRefreshQueryIssued(now, &scheduleState, plan.items[i]);
                                runtime->dmMotorCursor = (index + 1) % motorIds.size();
                                runtime->nextDmTick = std::chrono::steady_clock::now() +
                                                      clampRefreshSleep(protocol->refreshSleepInterval());
                                runtime->nextEligibleIssueTime = std::chrono::steady_clock::now() +
                                                                 runtime->interFrameGap;
                                runtime->pendingRefresh.protocol = CanType::DM;
                                runtime->pendingRefresh.motorId = systemMotorId;
                                runtime->pendingRefresh.observedLastRxSteadyNs =
                                    sharedAxisLastRxNs(sharedState, runtime->deviceName, CanType::DM, systemMotorId);
                                runtime->pendingRefresh.deadline = std::chrono::steady_clock::now() +
                                                                   clampRefreshSleep(protocol->refreshSleepInterval());
                                runtime->pendingRefresh.active = true;
                                runtime->protocolRoundRobinCursor = (static_cast<std::size_t>(AttemptedProtocol::Mt));
                            }
                        }
                    }
                }
            }

            if (!runtime->mtActive.load(std::memory_order_acquire)) {
                std::lock_guard<std::mutex> lock(runtime->scheduleMutex);
                runtime->nextMtTick = std::chrono::steady_clock::time_point {};
            }
            if (!runtime->ppActive.load(std::memory_order_acquire)) {
                std::lock_guard<std::mutex> lock(runtime->scheduleMutex);
                runtime->nextPpTick = std::chrono::steady_clock::time_point {};
            }
            if (!runtime->dmActive.load(std::memory_order_acquire)) {
                std::lock_guard<std::mutex> lock(runtime->scheduleMutex);
                runtime->nextDmTick = std::chrono::steady_clock::time_point {};
            }
        },
        [weakRuntime]() {
            const auto runtime = weakRuntime.lock();
            if (!runtime) {
                return std::chrono::milliseconds(5);
            }

            const auto now = std::chrono::steady_clock::now();
            std::chrono::milliseconds sleepFor(5);
            bool hasPending = false;
            {
                std::lock_guard<std::mutex> lock(runtime->scheduleMutex);
                if (runtime->mtActive.load(std::memory_order_acquire)) {
                    if (runtime->nextEligibleIssueTime != std::chrono::steady_clock::time_point {}) {
                        const auto frameGapWait = std::chrono::duration_cast<std::chrono::milliseconds>(
                            runtime->nextEligibleIssueTime > now ? (runtime->nextEligibleIssueTime - now)
                                                                 : std::chrono::steady_clock::duration::zero());
                        sleepFor = hasPending ? std::min(sleepFor, frameGapWait) : frameGapWait;
                        hasPending = true;
                    }
                    if (runtime->nextMtTick == std::chrono::steady_clock::time_point {}) {
                        return std::chrono::milliseconds(1);
                    }
                    const auto mtWait = std::chrono::duration_cast<std::chrono::milliseconds>(
                        runtime->nextMtTick > now ? (runtime->nextMtTick - now)
                                                  : std::chrono::steady_clock::duration::zero());
                    sleepFor = hasPending ? std::min(sleepFor, mtWait) : mtWait;
                    hasPending = true;
                }
                if (runtime->ppActive.load(std::memory_order_acquire)) {
                    if (runtime->nextPpTick == std::chrono::steady_clock::time_point {}) {
                        return std::chrono::milliseconds(1);
                    }
                    const auto ppWait = std::chrono::duration_cast<std::chrono::milliseconds>(
                        runtime->nextPpTick > now ? (runtime->nextPpTick - now)
                                                  : std::chrono::steady_clock::duration::zero());
                    sleepFor = hasPending ? std::min(sleepFor, ppWait) : ppWait;
                    hasPending = true;
                }
                if (runtime->dmActive.load(std::memory_order_acquire)) {
                    if (runtime->nextDmTick == std::chrono::steady_clock::time_point {}) {
                        return std::chrono::milliseconds(1);
                    }
                    const auto dmWait = std::chrono::duration_cast<std::chrono::milliseconds>(
                        runtime->nextDmTick > now ? (runtime->nextDmTick - now)
                                                  : std::chrono::steady_clock::duration::zero());
                    sleepFor = hasPending ? std::min(sleepFor, dmWait) : dmWait;
                    hasPending = true;
                }
            }

            if (!hasPending) {
                return std::chrono::milliseconds(5);
            }
            return clampRefreshSleep(sleepFor);
        });
}

void DeviceManager::startDeviceRefreshWorkerLocked(const std::string &device)
{
    const auto it = deviceRefreshRuntimes_.find(device);
    if (it == deviceRefreshRuntimes_.end() || !it->second) {
        return;
    }

    const auto &runtime = it->second;
    const bool anyActive = runtime->mtActive.load(std::memory_order_acquire) ||
                           runtime->ppActive.load(std::memory_order_acquire) ||
                           runtime->dmActive.load(std::memory_order_acquire);
    if (!anyActive || !runtime->worker) {
        return;
    }

    runtime->worker->start();
    runtime->worker->notify();
}

double DeviceManager::normalizeRefreshRateHz(double hz)
{
    return (std::isfinite(hz) && hz > 0.0) ? hz : 0.0;
}

double DeviceManager::effectiveRefreshRateHzLocked(const std::string &device) const
{
    const auto overrideIt = deviceRefreshRateOverrides_.find(device);
    if (overrideIt != deviceRefreshRateOverrides_.end()) {
        return overrideIt->second;
    }
    return refreshRateHz_;
}

void DeviceManager::applyRefreshRateLocked(const std::string &device, double hz)
{
    if (const auto mtIt = mtProtocols_.find(device); mtIt != mtProtocols_.end() && mtIt->second) {
        mtIt->second->setRefreshRateHz(hz);
    }
    if (const auto ppIt = eyouProtocols_.find(device);
        ppIt != eyouProtocols_.end() && ppIt->second) {
        ppIt->second->setRefreshRateHz(hz);
    }
    if (const auto dmIt = damiaoProtocols_.find(device);
        dmIt != damiaoProtocols_.end() && dmIt->second) {
        dmIt->second->setRefreshRateHz(hz);
    }
    if (const auto ecbIt = ecbProtocols_.find(device);
        ecbIt != ecbProtocols_.end() && ecbIt->second) {
        ecbIt->second->setRefreshRateHz(hz);
    }
    const auto runtimeIt = deviceRefreshRuntimes_.find(device);
    if (runtimeIt == deviceRefreshRuntimes_.end() || !runtimeIt->second) {
        return;
    }

    {
        std::lock_guard<std::mutex> scheduleLock(runtimeIt->second->scheduleMutex);
        runtimeIt->second->nextMtTick = std::chrono::steady_clock::time_point {};
        runtimeIt->second->nextPpTick = std::chrono::steady_clock::time_point {};
        runtimeIt->second->nextDmTick = std::chrono::steady_clock::time_point {};
    }
    if (runtimeIt->second->worker) {
        runtimeIt->second->worker->notify();
    }
}

void DeviceManager::stopDeviceRefreshWorkerLocked(const std::string &device)
{
    const auto it = deviceRefreshRuntimes_.find(device);
    if (it == deviceRefreshRuntimes_.end()) {
        return;
    }

    if (it->second && it->second->worker) {
        it->second->worker->stop();
    }
    deviceRefreshRuntimes_.erase(it);
}

void DeviceManager::stopAllDeviceRefreshWorkersLocked()
{
    for (auto &entry : deviceRefreshRuntimes_) {
        if (entry.second && entry.second->worker) {
            entry.second->worker->stop();
        }
    }
    deviceRefreshRuntimes_.clear();
}

void DeviceManager::resetDeviceRuntimeLocked(const std::string &device)
{
    mtProtocols_.erase(device);
    eyouProtocols_.erase(device);
    damiaoProtocols_.erase(device);
    ecbProtocols_.erase(device);
    txDispatchers_.erase(device);
}

void DeviceManager::shutdownDeviceLocked(const std::string &device)
{
    const auto socketIt = transports_.find(device);
    const auto socketTransport =
        (socketIt != transports_.end()) ? socketIt->second : std::shared_ptr<SocketCanController>();
    const auto transport = getTransportBaseLocked(device);

    if (sharedState_) {
        const auto stats = socketTransport ? socketTransport->snapshotStats()
                                           : SocketCanController::Stats {};
        sharedState_->mutateDeviceHealth(
            device,
            [&stats](can_driver::SharedDriverState::DeviceHealthState *health) {
                health->transportReady = false;
                health->txBackpressure = stats.txBackpressure;
                health->txLinkUnavailable = stats.txLinkUnavailable;
                health->txError = stats.txError;
                health->rxError = stats.rxError;
                health->lastTxLinkUnavailableSteadyNs = stats.lastTxLinkUnavailableSteadyNs;
                health->lastRxSteadyNs = stats.lastRxSteadyNs;
            });
    }

    stopDeviceRefreshWorkerLocked(device);
    if (transport) {
        shutdownTransportLocked(device);
    }
    resetDeviceRuntimeLocked(device);
    transports_.erase(device);
    udpTransports_.erase(device);
    ecbDevices_.erase(device);
    deviceCmdMutexes_.erase(device);
}

// 幂等创建 transport：同一 device 只创建一次。
bool DeviceManager::ensureTransport(const std::string &device, bool loopback)
{
    std::unique_lock<std::shared_mutex> lock(mutex_);
    if (isEcbDevice(device)) {
        ecbDevices_.insert(device);
        if (deviceCmdMutexes_.find(device) == deviceCmdMutexes_.end()) {
            deviceCmdMutexes_[device] = std::make_shared<std::mutex>();
        }
        return true;
    }

    if (getTransportBaseLocked(device)) {
        return true;
    }

    if (!initializeTransportLocked(device, loopback)) {
        return false;
    }

    auto transport = getTransportBaseLocked(device);
    txDispatchers_[device] = std::make_shared<DeviceRuntime>(transport, device);
    std::static_pointer_cast<DeviceRuntime>(txDispatchers_[device])->setSharedDriverState(sharedState_);
    if (sharedState_) {
        const auto socketIt = transports_.find(device);
        const auto stats = (socketIt != transports_.end() && socketIt->second)
                               ? socketIt->second->snapshotStats()
                               : SocketCanController::Stats {};
        sharedState_->mutateDeviceHealth(
            device,
            [&stats](can_driver::SharedDriverState::DeviceHealthState *health) {
                health->transportReady = true;
                health->txBackpressure = stats.txBackpressure;
                health->txLinkUnavailable = stats.txLinkUnavailable;
                health->txError = stats.txError;
                health->rxError = stats.rxError;
                health->lastTxLinkUnavailableSteadyNs = stats.lastTxLinkUnavailableSteadyNs;
                health->lastRxSteadyNs = stats.lastRxSteadyNs;
            });
    }
    // 同步创建该设备的命令互斥锁，供上层 write() 串行下发命令使用。
    deviceCmdMutexes_[device] = std::make_shared<std::mutex>();
    ROS_INFO("[CanDriverHW] Opened device '%s'.", device.c_str());
    return true;
}

bool DeviceManager::ensureProtocol(const std::string &device, CanType type)
{
    std::unique_lock<std::shared_mutex> lock(mutex_);
    if (type == CanType::ECB) {
        if (ecbProtocols_.find(device) == ecbProtocols_.end()) {
            ecbProtocols_[device] =
                std::make_shared<InnfosEcbProtocol>(device, sharedState_, device);
            ecbProtocols_[device]->setRefreshRateHz(effectiveRefreshRateHzLocked(device));
        }
        ecbDevices_.insert(device);
        if (deviceCmdMutexes_.find(device) == deviceCmdMutexes_.end()) {
            deviceCmdMutexes_[device] = std::make_shared<std::mutex>();
        }
        return true;
    }

    const auto transport = getTransportBaseLocked(device);
    if (!transport) {
        return false;
    }

    auto txDispatcherIt = txDispatchers_.find(device);
    if (txDispatcherIt == txDispatchers_.end()) {
        txDispatchers_[device] = std::make_shared<DeviceRuntime>(transport, device);
        txDispatcherIt = txDispatchers_.find(device);
    }
    auto txDispatcher = txDispatcherIt->second;

    // 按协议类型按需懒加载实例。
    if (type == CanType::MT) {
        if (mtProtocols_.find(device) == mtProtocols_.end()) {
            auto mt = std::make_shared<MtCan>(transport, txDispatcher, sharedState_, device);
            mt->setRefreshRateHz(effectiveRefreshRateHzLocked(device));
            mt->setCommunicationTimeoutOnInit(mtCommunicationTimeoutOnInitMs_);
            mtProtocols_[device] = std::move(mt);
        }
    } else if (type == CanType::PP) {
        if (eyouProtocols_.find(device) == eyouProtocols_.end()) {
            auto eyou =
                std::make_shared<EyouCan>(transport, txDispatcher, sharedState_, device);
            eyou->setRefreshRateHz(effectiveRefreshRateHzLocked(device));
            eyou->setFastWriteEnabled(ppFastWriteEnabled_);
            eyou->setDefaultPositionVelocityRaw(ppPositionDefaultVelocityRaw_);
            eyou->setDefaultCspVelocityRaw(ppCspDefaultVelocityRaw_);
            eyouProtocols_[device] = std::move(eyou);
        }
    } else if (type == CanType::DM) {
        if (damiaoProtocols_.find(device) == damiaoProtocols_.end()) {
            auto damiao =
                std::make_shared<DamiaoCan>(transport, txDispatcher, sharedState_, device);
            damiao->setRefreshRateHz(effectiveRefreshRateHzLocked(device));
            damiaoProtocols_[device] = std::move(damiao);
        }
    } else {
        return false;
    }
    return true;
}

bool DeviceManager::initDevice(const std::string &device,
                               const std::vector<std::pair<CanType, MotorID>> &motors,
                               bool loopback)
{
    std::unique_lock<std::shared_mutex> lock(mutex_);
    std::vector<MotorID> ecbIds;
    for (const auto &entry : motors) {
        if (entry.first == CanType::ECB) {
            ecbIds.push_back(entry.second);
        }
    }
    if (!ecbIds.empty() && isEcbDevice(device)) {
        if (ecbProtocols_.find(device) == ecbProtocols_.end()) {
            ecbProtocols_[device] =
                std::make_shared<InnfosEcbProtocol>(device, sharedState_, device);
        }
        ecbProtocols_[device]->setRefreshRateHz(effectiveRefreshRateHzLocked(device));
        ecbProtocols_[device]->initializeMotorRefresh(ecbIds);
        ecbDevices_.insert(device);
        if (deviceCmdMutexes_.find(device) == deviceCmdMutexes_.end()) {
            deviceCmdMutexes_[device] = std::make_shared<std::mutex>();
        }
        if (sharedState_) {
            for (const auto &entry : motors) {
                sharedState_->registerAxis(device, entry.first, entry.second);
            }
            sharedState_->mutateDeviceHealth(
                device,
                [](can_driver::SharedDriverState::DeviceHealthState *health) {
                    health->transportReady = true;
                });
        }
        ROS_INFO("[CanDriverHW] Initialized ECB device '%s'.", device.c_str());
        return true;
    }

    if (!getTransportBaseLocked(device)) {
        if (!initializeTransportLocked(device, loopback)) {
            ROS_ERROR("[CanDriverHW] Failed to init '%s'.", device.c_str());
            return false;
        }
    } else {
        stopDeviceRefreshWorkerLocked(device);
        shutdownTransportLocked(device);
        resetDeviceRuntimeLocked(device);
        if (!initializeTransportLocked(device, loopback)) {
            ROS_ERROR("[CanDriverHW] Re-init of '%s' failed.", device.c_str());
            shutdownDeviceLocked(device);
            return false;
        }
    }

    auto transport = getTransportBaseLocked(device);
    if (!transport) {
        ROS_ERROR("[CanDriverHW] Transport unavailable for '%s'.", device.c_str());
        return false;
    }

    if (deviceCmdMutexes_.find(device) == deviceCmdMutexes_.end()) {
        deviceCmdMutexes_[device] = std::make_shared<std::mutex>();
    }
    txDispatchers_[device] = std::make_shared<DeviceRuntime>(transport, device);
    std::static_pointer_cast<DeviceRuntime>(txDispatchers_[device])->setSharedDriverState(sharedState_);
    if (sharedState_) {
        for (const auto &entry : motors) {
            sharedState_->registerAxis(device, entry.first, entry.second);
        }
        const auto socketIt = transports_.find(device);
        const auto stats = (socketIt != transports_.end() && socketIt->second)
                               ? socketIt->second->snapshotStats()
                               : SocketCanController::Stats {};
        sharedState_->mutateDeviceHealth(
            device,
            [&stats](can_driver::SharedDriverState::DeviceHealthState *health) {
                health->transportReady = true;
                health->txBackpressure = stats.txBackpressure;
                health->txLinkUnavailable = stats.txLinkUnavailable;
                health->txError = stats.txError;
                health->rxError = stats.rxError;
                health->lastTxLinkUnavailableSteadyNs = stats.lastTxLinkUnavailableSteadyNs;
                health->lastRxSteadyNs = stats.lastRxSteadyNs;
            });
    }

    // 按协议拆分电机列表，避免不必要地创建协议实例。
    std::vector<MotorID> mtIds;
    std::vector<MotorID> ppIds;
    std::vector<MotorID> dmIds;
    for (const auto &entry : motors) {
        if (entry.first == CanType::MT) {
            mtIds.push_back(entry.second);
        } else if (entry.first == CanType::PP) {
            ppIds.push_back(entry.second);
        } else if (entry.first == CanType::DM) {
            dmIds.push_back(entry.second);
        }
    }
    // 初始化协议对象并启动状态刷新任务。
    if (!mtIds.empty() && mtProtocols_.find(device) == mtProtocols_.end()) {
        auto mt = std::make_shared<MtCan>(
            transport, txDispatchers_[device], sharedState_, device);
        mt->setRefreshRateHz(effectiveRefreshRateHzLocked(device));
        mt->setCommunicationTimeoutOnInit(mtCommunicationTimeoutOnInitMs_);
        mtProtocols_[device] = std::move(mt);
    }
    if (!ppIds.empty() && eyouProtocols_.find(device) == eyouProtocols_.end()) {
        auto eyou = std::make_shared<EyouCan>(
            transport, txDispatchers_[device], sharedState_, device);
        eyou->setRefreshRateHz(effectiveRefreshRateHzLocked(device));
        eyou->setFastWriteEnabled(ppFastWriteEnabled_);
        eyou->setDefaultPositionVelocityRaw(ppPositionDefaultVelocityRaw_);
        eyou->setDefaultCspVelocityRaw(ppCspDefaultVelocityRaw_);
        eyouProtocols_[device] = std::move(eyou);
    }
    if (!dmIds.empty() && damiaoProtocols_.find(device) == damiaoProtocols_.end()) {
        auto damiao = std::make_shared<DamiaoCan>(
            transport, txDispatchers_[device], sharedState_, device);
        damiao->setRefreshRateHz(effectiveRefreshRateHzLocked(device));
        damiaoProtocols_[device] = std::move(damiao);
    }

    syncDeviceRefreshRuntimeLocked(device);
    const auto runtime = deviceRefreshRuntimes_[device];

    if (!mtIds.empty()) {
        mtProtocols_[device]->initializeMotorRefresh(mtIds);
        runtime->mtActive.store(true, std::memory_order_release);
        std::lock_guard<std::mutex> lock(runtime->scheduleMutex);
        runtime->mtMotorIds = normalizeMtSystemMotorIds(mtIds);
        runtime->mtScheduleCycleCount = 0;
        runtime->nextMtTick = std::chrono::steady_clock::time_point {};
    } else {
        runtime->mtActive.store(false, std::memory_order_release);
        std::lock_guard<std::mutex> lock(runtime->scheduleMutex);
        runtime->mtMotorIds.clear();
        runtime->mtScheduleCycleCount = 0;
        runtime->nextMtTick = std::chrono::steady_clock::time_point {};
    }
    if (!ppIds.empty()) {
        eyouProtocols_[device]->initializeMotorRefresh(ppIds);
        runtime->ppActive.store(true, std::memory_order_release);
        std::lock_guard<std::mutex> lock(runtime->scheduleMutex);
        runtime->ppMotorIds = normalizeMotorIds(ppIds, CanType::PP);
        runtime->ppScheduleStates.clear();
        runtime->ppScheduleCycleCount = 0;
        runtime->nextPpTick = std::chrono::steady_clock::time_point {};
    } else {
        runtime->ppActive.store(false, std::memory_order_release);
        std::lock_guard<std::mutex> lock(runtime->scheduleMutex);
        runtime->ppMotorIds.clear();
        runtime->ppScheduleStates.clear();
        runtime->ppScheduleCycleCount = 0;
        runtime->nextPpTick = std::chrono::steady_clock::time_point {};
    }
    if (!dmIds.empty()) {
        damiaoProtocols_[device]->initializeMotorRefresh(dmIds);
        runtime->dmActive.store(true, std::memory_order_release);
        std::lock_guard<std::mutex> lock(runtime->scheduleMutex);
        runtime->dmMotorIds = normalizeMotorIds(dmIds, CanType::DM);
        runtime->dmScheduleStates.clear();
        runtime->nextDmTick = std::chrono::steady_clock::time_point {};
    } else {
        runtime->dmActive.store(false, std::memory_order_release);
        std::lock_guard<std::mutex> lock(runtime->scheduleMutex);
        runtime->dmMotorIds.clear();
        runtime->dmScheduleStates.clear();
        runtime->nextDmTick = std::chrono::steady_clock::time_point {};
    }
    startDeviceRefreshWorkerLocked(device);

    ROS_INFO("[CanDriverHW] Initialized '%s'.", device.c_str());
    return true;
}

void DeviceManager::setMtCommunicationTimeoutOnInitMs(uint32_t timeoutMs)
{
    std::unique_lock<std::shared_mutex> lock(mutex_);
    mtCommunicationTimeoutOnInitMs_ = timeoutMs;
    for (auto &entry : mtProtocols_) {
        if (entry.second) {
            entry.second->setCommunicationTimeoutOnInit(timeoutMs);
        }
    }
}

void DeviceManager::startRefresh(const std::string &device,
                                 CanType type,
                                 const std::vector<MotorID> &ids)
{
    std::unique_lock<std::shared_mutex> lock(mutex_);
    if (type == CanType::MT) {
        auto it = mtProtocols_.find(device);
        if (it != mtProtocols_.end()) {
            it->second->initializeMotorRefresh(ids);
            syncDeviceRefreshRuntimeLocked(device);
            auto runtime = deviceRefreshRuntimes_[device];
            runtime->mtActive.store(!ids.empty(), std::memory_order_release);
            const auto normalizedIds = normalizeMtSystemMotorIds(ids);
            {
                std::lock_guard<std::mutex> scheduleLock(runtime->scheduleMutex);
                if (runtime->mtMotorIds != normalizedIds) {
                    runtime->mtMotorIds = normalizedIds;
                    runtime->mtScheduleCycleCount = 0;
                }
                runtime->nextMtTick = std::chrono::steady_clock::time_point {};
            }
            if (runtime->mtActive.load(std::memory_order_acquire) ||
                runtime->ppActive.load(std::memory_order_acquire) ||
                runtime->dmActive.load(std::memory_order_acquire)) {
                startDeviceRefreshWorkerLocked(device);
            } else {
                stopDeviceRefreshWorkerLocked(device);
            }
        }
    } else if (type == CanType::PP) {
        auto it = eyouProtocols_.find(device);
        if (it != eyouProtocols_.end()) {
            it->second->initializeMotorRefresh(ids);
            syncDeviceRefreshRuntimeLocked(device);
            auto runtime = deviceRefreshRuntimes_[device];
            runtime->ppActive.store(!ids.empty(), std::memory_order_release);
            const auto normalizedIds = normalizeMotorIds(ids, CanType::PP);
            {
                std::lock_guard<std::mutex> scheduleLock(runtime->scheduleMutex);
                if (runtime->ppMotorIds != normalizedIds) {
                    runtime->ppMotorIds = normalizedIds;
                    runtime->ppScheduleStates.clear();
                    runtime->ppScheduleCycleCount = 0;
                }
                runtime->nextPpTick = std::chrono::steady_clock::time_point {};
            }
            if (runtime->mtActive.load(std::memory_order_acquire) ||
                runtime->ppActive.load(std::memory_order_acquire) ||
                runtime->dmActive.load(std::memory_order_acquire)) {
                startDeviceRefreshWorkerLocked(device);
            } else {
                stopDeviceRefreshWorkerLocked(device);
            }
        }
    } else if (type == CanType::DM) {
        auto it = damiaoProtocols_.find(device);
        if (it != damiaoProtocols_.end()) {
            it->second->initializeMotorRefresh(ids);
            syncDeviceRefreshRuntimeLocked(device);
            auto runtime = deviceRefreshRuntimes_[device];
            runtime->dmActive.store(!ids.empty(), std::memory_order_release);
            const auto normalizedIds = normalizeMotorIds(ids, CanType::DM);
            {
                std::lock_guard<std::mutex> scheduleLock(runtime->scheduleMutex);
                if (runtime->dmMotorIds != normalizedIds) {
                    runtime->dmMotorIds = normalizedIds;
                    runtime->dmScheduleStates.clear();
                }
                runtime->nextDmTick = std::chrono::steady_clock::time_point {};
            }
            if (runtime->mtActive.load(std::memory_order_acquire) ||
                runtime->ppActive.load(std::memory_order_acquire) ||
                runtime->dmActive.load(std::memory_order_acquire)) {
                startDeviceRefreshWorkerLocked(device);
            } else {
                stopDeviceRefreshWorkerLocked(device);
            }
        }
    } else if (type == CanType::ECB) {
        auto it = ecbProtocols_.find(device);
        if (it != ecbProtocols_.end()) {
            it->second->initializeMotorRefresh(ids);
        }
    }
}

void DeviceManager::setRefreshRateHz(double hz)
{
    std::unique_lock<std::shared_mutex> lock(mutex_);
    refreshRateHz_ = normalizeRefreshRateHz(hz);
    for (const auto &kv : deviceCmdMutexes_) {
        applyRefreshRateLocked(kv.first, effectiveRefreshRateHzLocked(kv.first));
    }
}

void DeviceManager::setDeviceRefreshRateHz(const std::string &device, double hz)
{
    std::unique_lock<std::shared_mutex> lock(mutex_);
    const double normalizedHz = normalizeRefreshRateHz(hz);
    if (normalizedHz > 0.0) {
        deviceRefreshRateOverrides_[device] = normalizedHz;
    } else {
        deviceRefreshRateOverrides_.erase(device);
    }

    if (deviceCmdMutexes_.find(device) == deviceCmdMutexes_.end()) {
        return;
    }
    applyRefreshRateLocked(device, effectiveRefreshRateHzLocked(device));
}

void DeviceManager::setPpFastWriteEnabled(bool enabled)
{
    std::unique_lock<std::shared_mutex> lock(mutex_);
    ppFastWriteEnabled_ = enabled;
    for (auto &kv : eyouProtocols_) {
        if (kv.second) {
            kv.second->setFastWriteEnabled(enabled);
        }
    }
}

void DeviceManager::setPpDefaultPositionVelocityRaw(int32_t velocityRaw)
{
    setPpPositionDefaultVelocityRaw(velocityRaw);
    setPpCspDefaultVelocityRaw(velocityRaw);
}

void DeviceManager::setPpPositionDefaultVelocityRaw(int32_t velocityRaw)
{
    std::unique_lock<std::shared_mutex> lock(mutex_);
    if (velocityRaw <= 0) {
        ROS_WARN("[CanDriverHW] Ignore invalid pp_position_default_velocity_raw=%d.", velocityRaw);
        return;
    }
    ppPositionDefaultVelocityRaw_ = velocityRaw;
    for (auto &kv : eyouProtocols_) {
        if (kv.second) {
            kv.second->setDefaultPositionVelocityRaw(velocityRaw);
        }
    }
}

void DeviceManager::setPpCspDefaultVelocityRaw(int32_t velocityRaw)
{
    std::unique_lock<std::shared_mutex> lock(mutex_);
    if (velocityRaw <= 0) {
        ROS_WARN("[CanDriverHW] Ignore invalid pp_csp_default_velocity_raw=%d.", velocityRaw);
        return;
    }
    ppCspDefaultVelocityRaw_ = velocityRaw;
    for (auto &kv : eyouProtocols_) {
        if (kv.second) {
            kv.second->setDefaultCspVelocityRaw(velocityRaw);
        }
    }
}

void DeviceManager::shutdownAll()
{
    std::unique_lock<std::shared_mutex> lock(mutex_);
    std::vector<std::string> devices;
    devices.reserve(transports_.size() + udpTransports_.size() + ecbDevices_.size());
    for (const auto &kv : transports_) {
        devices.push_back(kv.first);
    }
    for (const auto &kv : udpTransports_) {
        devices.push_back(kv.first);
    }
    for (const auto &device : ecbDevices_) {
        devices.push_back(device);
    }

    for (const auto &device : devices) {
        shutdownDeviceLocked(device);
    }

    stopAllDeviceRefreshWorkersLocked();
    mtProtocols_.clear();
    eyouProtocols_.clear();
    damiaoProtocols_.clear();
    ecbProtocols_.clear();
    txDispatchers_.clear();
    transports_.clear();
    udpTransports_.clear();
    ecbDevices_.clear();
    deviceCmdMutexes_.clear();
}

void DeviceManager::shutdownDevice(const std::string &device)
{
    std::unique_lock<std::shared_mutex> lock(mutex_);
    shutdownDeviceLocked(device);
}

std::shared_ptr<CanProtocol> DeviceManager::getProtocol(const std::string &device, CanType type) const
{
    std::shared_lock<std::shared_mutex> lock(mutex_);
    if (type == CanType::MT) {
        auto it = mtProtocols_.find(device);
        if (it != mtProtocols_.end()) {
            return std::static_pointer_cast<CanProtocol>(it->second);
        }
    } else if (type == CanType::PP) {
        auto it = eyouProtocols_.find(device);
        if (it != eyouProtocols_.end()) {
            return std::static_pointer_cast<CanProtocol>(it->second);
        }
    } else if (type == CanType::DM) {
        auto it = damiaoProtocols_.find(device);
        if (it != damiaoProtocols_.end()) {
            return std::static_pointer_cast<CanProtocol>(it->second);
        }
    } else if (type == CanType::ECB) {
        auto it = ecbProtocols_.find(device);
        if (it != ecbProtocols_.end()) {
            return std::static_pointer_cast<CanProtocol>(it->second);
        }
    }
    return nullptr;
}

std::shared_ptr<std::mutex> DeviceManager::getDeviceMutex(const std::string &device) const
{
    std::shared_lock<std::shared_mutex> lock(mutex_);
    auto it = deviceCmdMutexes_.find(device);
    return (it != deviceCmdMutexes_.end()) ? it->second : nullptr;
}

std::shared_ptr<SocketCanController> DeviceManager::getTransport(const std::string &device) const
{
    std::shared_lock<std::shared_mutex> lock(mutex_);
    auto it = transports_.find(device);
    return (it != transports_.end()) ? it->second : nullptr;
}

bool DeviceManager::isDeviceReady(const std::string &device) const
{
    std::shared_lock<std::shared_mutex> lock(mutex_);
    if (isEcbDevice(device)) {
        return ecbProtocols_.find(device) != ecbProtocols_.end();
    }

    bool ready = false;
    SocketCanController::Stats stats {};
    const auto canIt = transports_.find(device);
    if (canIt != transports_.end() && canIt->second) {
        stats = canIt->second->snapshotStats();
        const bool linkRecovered =
            stats.lastTxLinkUnavailableSteadyNs == 0 ||
            stats.lastRxSteadyNs > stats.lastTxLinkUnavailableSteadyNs;
        ready = canIt->second->isReady() && linkRecovered;
    } else {
        const auto udpIt = udpTransports_.find(device);
        if (udpIt != udpTransports_.end() && udpIt->second) {
            ready = udpIt->second->isReady();
        }
    }

    if (sharedState_) {
        sharedState_->mutateDeviceHealth(
            device,
            [ready, &stats](can_driver::SharedDriverState::DeviceHealthState *health) {
                health->transportReady = ready;
                health->txBackpressure = stats.txBackpressure;
                health->txLinkUnavailable = stats.txLinkUnavailable;
                health->txError = stats.txError;
                health->rxError = stats.rxError;
                health->lastTxLinkUnavailableSteadyNs = stats.lastTxLinkUnavailableSteadyNs;
                health->lastRxSteadyNs = stats.lastRxSteadyNs;
            });
    }
    return ready;
}

std::size_t DeviceManager::deviceCount() const
{
    std::shared_lock<std::shared_mutex> lock(mutex_);
    return transports_.size() + udpTransports_.size() + ecbDevices_.size();
}

std::shared_ptr<can_driver::SharedDriverState> DeviceManager::getSharedDriverState() const
{
    std::shared_lock<std::shared_mutex> lock(mutex_);
    return sharedState_;
}
