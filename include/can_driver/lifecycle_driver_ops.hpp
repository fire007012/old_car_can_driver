#ifndef CAN_DRIVER_LIFECYCLE_DRIVER_OPS_HPP
#define CAN_DRIVER_LIFECYCLE_DRIVER_OPS_HPP

#include "can_driver/AxisReadinessEvaluator.h"
#include "can_driver/IDeviceManager.h"
#include "can_driver/motor_action_executor.hpp"
#include "can_driver/operational_coordinator.hpp"

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <cstdint>
#include <vector>

namespace can_driver {

class LifecycleDriverOps {
public:
    using Result = OperationalCoordinator::Result;

    LifecycleDriverOps() = default;
    LifecycleDriverOps(std::shared_ptr<IDeviceManager> deviceManager,
                       const MotorActionExecutor *motorActionExecutor);

    void configure(std::shared_ptr<IDeviceManager> deviceManager,
                   const MotorActionExecutor *motorActionExecutor);
    void setTargets(std::vector<MotorActionExecutor::Target> targets);
    void setFeedbackFreshnessTimeoutNs(std::int64_t timeoutNs);

    Result prepareDevice(const std::string &device, bool loopback) const;
    Result enableDevice(const std::string &device) const;
    Result initializeDevice(const std::string &device, bool loopback) const;
    Result shutdownDevice(const std::string &device) const;
    Result enableAll() const;
    Result disableAll() const;
    Result haltAll() const;
    Result recoverAll() const;
    Result shutdownAll(bool force) const;
    bool enableHealthy(std::string *detail) const;
    bool anyFaultActive() const;
    bool motionHealthy(std::string *detail) const;

private:
    struct AxisRecoverTracker {
        std::uint32_t healthyCycles{0};
        std::int64_t lastSampleNs{0};
    };

    std::vector<MotorActionExecutor::Target> targetsSnapshot() const;
    std::vector<MotorActionExecutor::Target> lifecycleTargetsSnapshot() const;
    void activateTargetsForDevice(
        const std::string &device,
        const std::vector<MotorActionExecutor::Target> &deviceTargets) const;
    void deactivateTargetsForDevice(const std::string &device) const;
    void clearActiveTargets() const;
    Result makeMotorActionFailureResult(MotorActionExecutor::Status status,
                                        const char *rejectedMessage,
                                        const char *protocolUnavailableMessage) const;
    Result runMotorBatchAction(const std::vector<MotorActionExecutor::Target> &targets,
                               const MotorActionExecutor::Action &action,
                               const char *operationName,
                               const char *rejectedMessage,
                               const char *protocolUnavailableMessage,
                               bool requireAnyTarget) const;
    Result waitForEnabledTargets(const std::vector<MotorActionExecutor::Target> &targets,
                                 std::chrono::milliseconds timeout,
                                 std::chrono::milliseconds pollInterval) const;
    bool queryMotorFault(const MotorActionExecutor::Target &target, bool *hasFault) const;
    bool queryMotorEnabled(const MotorActionExecutor::Target &target, bool *enabled) const;
    bool enableHealthyOnce(std::string *detail) const;
    bool motionHealthyOnce(std::string *detail) const;
    std::shared_ptr<CanProtocol> getProtocol(const std::string &device, CanType type) const;
    std::shared_ptr<std::mutex> getDeviceMutex(const std::string &device) const;
    bool isDeviceReady(const std::string &device) const;
    std::shared_ptr<SharedDriverState> getSharedDriverState() const;
    AxisReadiness evaluateAxisReadiness(
        const SharedDriverState::AxisKey &axisKey,
        const SharedDriverState::AxisFeedbackState &feedback,
        const SharedDriverState::AxisCommandState *command,
        AxisIntent intent,
        const SharedDriverState::DeviceHealthState *deviceHealth) const;

    std::shared_ptr<IDeviceManager> deviceManager_;
    const MotorActionExecutor *motorActionExecutor_{nullptr};
    mutable std::mutex targetsMutex_;
    mutable std::vector<MotorActionExecutor::Target> activeTargets_;
    mutable std::mutex axisReadinessMutex_;
    AxisReadinessEvaluator axisReadinessEvaluator_{};
    mutable std::map<std::string, AxisRecoverTracker> axisRecoverTrackers_;
    std::vector<MotorActionExecutor::Target> targets_;
};

} // namespace can_driver

#endif // CAN_DRIVER_LIFECYCLE_DRIVER_OPS_HPP
