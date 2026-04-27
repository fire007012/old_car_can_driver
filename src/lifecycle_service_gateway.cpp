#include "can_driver/lifecycle_service_gateway.hpp"

#include "can_driver/operational_coordinator.hpp"

namespace {

constexpr uint16_t kGlobalRecoverMotorId = 0xFFFFu;

bool isAlreadyMessage(const std::string &message)
{
    return message.rfind("already ", 0) == 0;
}

std::string selectSuccessMessage(const can_driver::OperationalCoordinator::Result &result,
                                 const char *canonicalSuccess,
                                 const char *canonicalAlready = nullptr)
{
    if (isAlreadyMessage(result.message)) {
        return canonicalAlready ? canonicalAlready : result.message;
    }
    if (canonicalSuccess != nullptr && canonicalSuccess[0] != '\0') {
        return canonicalSuccess;
    }
    return result.message;
}

std::string selectFailureMessage(const can_driver::OperationalCoordinator::Result &result,
                                 const char *fallback)
{
    return result.message.empty() ? std::string(fallback) : result.message;
}

template <typename Response>
bool ensureGatewayReady(can_driver::OperationalCoordinator *coordinator, Response &res)
{
    if (coordinator != nullptr) {
        return true;
    }

    res.success = false;
    res.message = "lifecycle gateway not initialized";
    return false;
}

} // namespace

LifecycleServiceGateway::LifecycleServiceGateway(
    ros::NodeHandle &pnh,
    can_driver::OperationalCoordinator *coordinator)
{
    initialize(pnh, coordinator);
}

void LifecycleServiceGateway::initialize(
    ros::NodeHandle &pnh,
    can_driver::OperationalCoordinator *coordinator)
{
    coordinator_ = coordinator;
    initSrv_ = pnh.advertiseService("init", &LifecycleServiceGateway::onInit, this);
    shutdownSrv_ = pnh.advertiseService("shutdown", &LifecycleServiceGateway::onShutdown, this);
    recoverSrv_ = pnh.advertiseService("recover", &LifecycleServiceGateway::onRecover, this);
    enableSrv_ = pnh.advertiseService("enable", &LifecycleServiceGateway::onEnable, this);
    disableSrv_ = pnh.advertiseService("disable", &LifecycleServiceGateway::onDisable, this);
    haltSrv_ = pnh.advertiseService("halt", &LifecycleServiceGateway::onHalt, this);
    resumeSrv_ = pnh.advertiseService("resume", &LifecycleServiceGateway::onResume, this);
}

bool LifecycleServiceGateway::onInit(can_driver::Init::Request &req,
                                     can_driver::Init::Response &res)
{
    if (!ensureGatewayReady(coordinator_, res)) {
        return true;
    }
    const auto result = coordinator_->RequestInit(req.device, req.loopback);
    res.success = result.ok;
    res.message = result.ok
                      ? selectSuccessMessage(result, "initialized (standby)", "already initialized")
                      : selectFailureMessage(result, "init failed");
    return true;
}

bool LifecycleServiceGateway::onShutdown(can_driver::Shutdown::Request &req,
                                         can_driver::Shutdown::Response &res)
{
    if (!ensureGatewayReady(coordinator_, res)) {
        return true;
    }
    const auto result = coordinator_->RequestShutdown(req.force);
    res.success = result.ok;
    res.message = result.ok ? "communication stopped; call ~/init first"
                            : selectFailureMessage(result, "shutdown failed");
    return true;
}

bool LifecycleServiceGateway::onRecover(can_driver::Recover::Request &req,
                                        can_driver::Recover::Response &res)
{
    if (!ensureGatewayReady(coordinator_, res)) {
        return true;
    }
    if (req.motor_id != kGlobalRecoverMotorId) {
        res.success = false;
        res.message = "per-motor recover has been removed; use motor_id=65535 for global recover";
        return true;
    }
    const auto result = coordinator_->RequestRecover();
    res.success = result.ok;
    res.message = result.ok ? selectSuccessMessage(result, "recovered (standby)")
                            : selectFailureMessage(result, "recover failed");
    return true;
}

bool LifecycleServiceGateway::onEnable(std_srvs::Trigger::Request &req,
                                       std_srvs::Trigger::Response &res)
{
    if (!ensureGatewayReady(coordinator_, res)) {
        return true;
    }
    const auto result = coordinator_->RequestEnable();
    res.success = result.ok;
    res.message = result.ok
                      ? selectSuccessMessage(result, "enabled (armed)", "already enabled")
                      : selectFailureMessage(result, "enable failed");
    return true;
}

bool LifecycleServiceGateway::onDisable(std_srvs::Trigger::Request &req,
                                        std_srvs::Trigger::Response &res)
{
    if (!ensureGatewayReady(coordinator_, res)) {
        return true;
    }
    const auto result = coordinator_->RequestDisable();
    res.success = result.ok;
    res.message = result.ok
                      ? selectSuccessMessage(result, "disabled (standby)", "already disabled")
                      : selectFailureMessage(result, "disable failed");
    return true;
}

bool LifecycleServiceGateway::onHalt(std_srvs::Trigger::Request &req,
                                     std_srvs::Trigger::Response &res)
{
    if (!ensureGatewayReady(coordinator_, res)) {
        return true;
    }
    const auto result = coordinator_->RequestHalt();
    res.success = result.ok;
    res.message = result.ok
                      ? selectSuccessMessage(result, "halted", "already halted")
                      : selectFailureMessage(result, "halt failed");
    return true;
}

bool LifecycleServiceGateway::onResume(std_srvs::Trigger::Request &req,
                                       std_srvs::Trigger::Response &res)
{
    if (!ensureGatewayReady(coordinator_, res)) {
        return true;
    }
    const auto result = coordinator_->RequestRelease();
    res.success = result.ok;
    res.message = result.ok
                      ? selectSuccessMessage(result, "resumed", "already running")
                      : selectFailureMessage(
                            result,
                            "resume failed; call ~/enable (or ~/recover then ~/enable) first");
    return true;
}
