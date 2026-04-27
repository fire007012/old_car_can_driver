#include "can_driver/CanDriverRuntime.h"

#include <cctype>
#include <sstream>
#include <utility>
#include <vector>

#include <ros/ros.h>

namespace can_driver {

namespace {

std::string trimCopy(const std::string& input)
{
    std::size_t begin = 0;
    while (begin < input.size() && std::isspace(static_cast<unsigned char>(input[begin])) != 0) {
        ++begin;
    }
    std::size_t end = input.size();
    while (end > begin && std::isspace(static_cast<unsigned char>(input[end - 1])) != 0) {
        --end;
    }
    return input.substr(begin, end - begin);
}

} // namespace

CanDriverRuntime::CanDriverRuntime()
    : deviceManager_(std::make_shared<DeviceManager>())
{
    configureDependencies();
}

CanDriverRuntime::CanDriverRuntime(std::shared_ptr<IDeviceManager> deviceManager)
    : deviceManager_(std::move(deviceManager))
{
    if (!deviceManager_) {
        deviceManager_ = std::make_shared<DeviceManager>();
    }
    configureDependencies();
}

void CanDriverRuntime::configureDependencies()
{
    motorActionExecutor_.setDeviceManager(deviceManager_);
    lifecycleDriverOps_.configure(deviceManager_, &motorActionExecutor_);
}

void CanDriverRuntime::reset()
{
    active_.store(false, std::memory_order_release);
    lifecycleCoordinator_.SetInactive();
    deviceLoopbackByName_.clear();
    commandGate_.reset();
    lifecycleDriverOps_.setTargets({});
    if (deviceManager_) {
        deviceManager_->shutdownAll();
    }
}

void CanDriverRuntime::configureCommandGate(
    std::function<std::vector<CommandGate::Snapshot>()> snapshotProvider,
    std::function<void()> holdCallback)
{
    commandGate_.configure(std::move(snapshotProvider), std::move(holdCallback));
}

void CanDriverRuntime::configureLifecycleCoordinator(LifecycleHooks hooks)
{
    lifecycleHooks_ = std::move(hooks);

    OperationalCoordinator::DriverOps ops;
    ops.init_device = [this](const std::string &device, bool loopback) {
        return initializeLifecycleDevice(device, loopback);
    };
    ops.enable_all = [this]() {
        return lifecycleDriverOps_.enableAll();
    };
    ops.enable_healthy = [this](std::string *detail) {
        return lifecycleHooks_.enable_healthy ? lifecycleHooks_.enable_healthy(detail)
                                              : lifecycleDriverOps_.enableHealthy(detail);
    };
    ops.disable_all = [this]() {
        return lifecycleDriverOps_.disableAll();
    };
    ops.halt_all = [this]() {
        return lifecycleDriverOps_.haltAll();
    };
    ops.recover_all = [this]() {
        return recoverLifecycleDevices();
    };
    ops.shutdown_all = [this](bool force) {
        return shutdownLifecycleDriver(force);
    };
    ops.motion_healthy = [this](std::string *detail) {
        return lifecycleHooks_.motion_healthy ? lifecycleHooks_.motion_healthy(detail)
                                              : lifecycleDriverOps_.motionHealthy(detail);
    };
    ops.any_fault_active = [this]() {
        return lifecycleDriverOps_.anyFaultActive();
    };
    ops.hold_commands = [this]() {
        commandGate_.holdCommands();
    };
    ops.arm_fresh_command_latch = [this]() {
        commandGate_.armFreshCommandLatch();
    };
    lifecycleCoordinator_.SetDriverOps(std::move(ops));
}

OperationalCoordinator::Result CanDriverRuntime::initializeLifecycleDevice(
    const std::string &device,
    bool loopback)
{
    std::vector<std::pair<std::string, bool>> specs;
    {
        std::stringstream ss(device);
        std::string token;
        while (std::getline(ss, token, ',')) {
            token = trimCopy(token);
            if (token.empty()) {
                continue;
            }
            bool optional = false;
            const std::string optionalPrefix("optional:");
            if (token.rfind(optionalPrefix, 0) == 0) {
                optional = true;
                token = trimCopy(token.substr(optionalPrefix.size()));
                if (token.empty()) {
                    continue;
                }
            }
            specs.emplace_back(token, optional);
        }
    }

    if (specs.empty()) {
        specs.emplace_back(device, false);
    }

    std::vector<std::string> initializedDevices;
    std::vector<std::string> optionalFailures;

    for (const auto& spec : specs) {
        const std::string& targetDevice = spec.first;
        const bool optional = spec.second;

        if (targetDevice.empty()) {
            continue;
        }

        deviceLoopbackByName_[targetDevice] = loopback;

        const auto prepareResult = prepareLifecycleDeviceForStandby(targetDevice, loopback);
        if (!prepareResult.ok) {
            if (optional) {
                ROS_WARN("[CanDriverRuntime] Optional device '%s' prepare failed: %s",
                         targetDevice.c_str(),
                         prepareResult.message.c_str());
                optionalFailures.push_back(targetDevice + "(prepare: " + prepareResult.message + ")");
                continue;
            }

            for (const auto& rollbackDevice : initializedDevices) {
                const auto rollback = lifecycleDriverOps_.shutdownDevice(rollbackDevice);
                if (!rollback.ok) {
                    ROS_ERROR("[CanDriverRuntime] Failed to rollback initialized device '%s': %s",
                              rollbackDevice.c_str(),
                              rollback.message.c_str());
                }
            }
            return prepareResult;
        }

        initializedDevices.push_back(targetDevice);
    }

    if (initializedDevices.empty()) {
        if (!optionalFailures.empty()) {
            std::ostringstream oss;
            oss << "all selected devices failed (optional): ";
            for (std::size_t i = 0; i < optionalFailures.size(); ++i) {
                if (i > 0U) {
                    oss << "; ";
                }
                oss << optionalFailures[i];
            }
            return {false, oss.str()};
        }
        return {false, "no device initialized"};
    }

    active_.store(true, std::memory_order_release);

    std::ostringstream msg;
    msg << "initialized (standby): ";
    for (std::size_t i = 0; i < initializedDevices.size(); ++i) {
        if (i > 0U) {
            msg << ", ";
        }
        msg << initializedDevices[i];
    }
    if (!optionalFailures.empty()) {
        msg << "; optional failures: ";
        for (std::size_t i = 0; i < optionalFailures.size(); ++i) {
            if (i > 0U) {
                msg << "; ";
            }
            msg << optionalFailures[i];
        }
    }
    return {true, msg.str()};
}

OperationalCoordinator::Result CanDriverRuntime::prepareLifecycleDeviceForStandby(
    const std::string &device,
    bool loopback)
{
    if (!lifecycleHooks_.sync_startup_position_and_commands ||
        !lifecycleHooks_.apply_pp_default_velocities ||
        !lifecycleHooks_.apply_initial_modes) {
        return {false, "prepare lifecycle path not available"};
    }

    const auto rollbackPreparedDevice =
        [this, &device](const OperationalCoordinator::Result &failure) {
            const auto rollback = lifecycleDriverOps_.shutdownDevice(device);
            if (!rollback.ok) {
                ROS_ERROR("[CanDriverRuntime] Failed to roll back prepared device '%s' after init failure: %s",
                          device.c_str(),
                          rollback.message.c_str());
            }
            return failure;
        };

    const double startupQueryHz =
        lifecycleHooks_.startup_query_hz ? lifecycleHooks_.startup_query_hz() : 0.0;
    const auto restoreSteadyRefresh = [this, &device]() {
        if (lifecycleHooks_.set_device_refresh_rate) {
            lifecycleHooks_.set_device_refresh_rate(device, 0.0);
        }
    };
    if (lifecycleHooks_.set_device_refresh_rate) {
        lifecycleHooks_.set_device_refresh_rate(device, startupQueryHz);
    }

    const auto prepareResult = lifecycleDriverOps_.prepareDevice(device, loopback);
    if (!prepareResult.ok) {
        restoreSteadyRefresh();
        return prepareResult;
    }

    if (lifecycleHooks_.apply_persisted_pp_zero_offsets &&
        !lifecycleHooks_.apply_persisted_pp_zero_offsets(device)) {
        restoreSteadyRefresh();
        return rollbackPreparedDevice(
            {false, "Failed to restore persisted PP zero offsets on " + device});
    }
    if (!lifecycleHooks_.sync_startup_position_and_commands(device)) {
        restoreSteadyRefresh();
        return rollbackPreparedDevice(
            {false, "Failed to synchronize startup position on " + device});
    }
    if (!lifecycleHooks_.apply_pp_default_velocities(device)) {
        restoreSteadyRefresh();
        return rollbackPreparedDevice(
            {false, "Failed to configure PP default velocities on " + device});
    }
    if (!lifecycleHooks_.apply_initial_modes(device)) {
        restoreSteadyRefresh();
        return rollbackPreparedDevice({false, "Failed to apply initial modes on " + device});
    }

    restoreSteadyRefresh();
    return {true, "prepared (standby)"};
}

OperationalCoordinator::Result CanDriverRuntime::recoverLifecycleDevices()
{
    if (!lifecycleHooks_.recover_devices) {
        return {false, "recover lifecycle path not available"};
    }

    const auto devices = lifecycleHooks_.recover_devices();
    if (devices.empty()) {
        return {false, "No devices available for recover."};
    }

    for (const auto &device : devices) {
        const auto loopbackIt = deviceLoopbackByName_.find(device);
        const bool loopback =
            (loopbackIt != deviceLoopbackByName_.end()) ? loopbackIt->second : false;
        const auto prepareResult = prepareLifecycleDeviceForStandby(device, loopback);
        if (!prepareResult.ok) {
            return prepareResult;
        }
    }

    const auto recoverResult = lifecycleDriverOps_.recoverAll();
    if (!recoverResult.ok) {
        return recoverResult;
    }

    active_.store(true, std::memory_order_release);
    return {true, "recovered (standby)"};
}

OperationalCoordinator::Result CanDriverRuntime::shutdownLifecycleDriver(bool force)
{
    active_.store(false, std::memory_order_release);
    const auto result = lifecycleDriverOps_.shutdownAll(force);

    if (lifecycleHooks_.clear_command_state) {
        lifecycleHooks_.clear_command_state();
    }

    if (result.ok) {
        ROS_INFO("[CanDriverRuntime] All devices shut down.");
    }
    return result;
}

} // namespace can_driver
