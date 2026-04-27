#include "can_driver/operational_coordinator.hpp"

#include <sstream>

#include <ros/ros.h>

namespace can_driver {

OperationalCoordinator::OperationalCoordinator(DriverOps driverOps)
    : driverOps_(std::move(driverOps))
{
}

const char *SystemOpModeName(SystemOpMode mode)
{
    switch (mode) {
    case SystemOpMode::Inactive:
        return "Inactive";
    case SystemOpMode::Configured:
        return "Configured";
    case SystemOpMode::Standby:
        return "Standby";
    case SystemOpMode::Armed:
        return "Armed";
    case SystemOpMode::Running:
        return "Running";
    case SystemOpMode::Faulted:
        return "Faulted";
    case SystemOpMode::Recovering:
        return "Recovering";
    case SystemOpMode::ShuttingDown:
        return "ShuttingDown";
    }
    return "Unknown";
}

void OperationalCoordinator::ForceMode(SystemOpMode to, const char *reason)
{
    const SystemOpMode from = mode_.exchange(to, std::memory_order_acq_rel);
    if (from == to) {
        return;
    }

    if (reason && reason[0] != '\0') {
        ROS_INFO("[can_driver::OperationalCoordinator] %s -> %s (%s)",
                 SystemOpModeName(from), SystemOpModeName(to), reason);
    } else {
        ROS_INFO("[can_driver::OperationalCoordinator] %s -> %s",
                 SystemOpModeName(from), SystemOpModeName(to));
    }
}

void OperationalCoordinator::SetInactive()
{
    ForceMode(SystemOpMode::Inactive, "force");
}

void OperationalCoordinator::SetConfigured()
{
    ForceMode(SystemOpMode::Configured, "configured");
}

void OperationalCoordinator::SetFaulted()
{
    ForceMode(SystemOpMode::Faulted, "faulted");
}

void OperationalCoordinator::SetDriverOps(DriverOps driverOps)
{
    std::lock_guard<std::mutex> lock(transitionMutex_);
    driverOps_ = std::move(driverOps);
}

OperationalCoordinator::Result OperationalCoordinator::DoTransition(
    std::initializer_list<SystemOpMode> allowedFrom,
    SystemOpMode to,
    const std::function<bool(std::string *)> &action)
{
    std::lock_guard<std::mutex> lock(transitionMutex_);

    const SystemOpMode current = mode_.load(std::memory_order_acquire);
    bool allowed = false;
    for (const auto from : allowedFrom) {
        if (from == current) {
            allowed = true;
            break;
        }
    }

    if (!allowed) {
        if (current == to) {
            return {true, std::string("already ") + SystemOpModeName(to)};
        }

        std::ostringstream oss;
        oss << "cannot transition from " << SystemOpModeName(current)
            << " to " << SystemOpModeName(to);
        return {false, oss.str()};
    }

    std::string detail;
    if (action && !action(&detail)) {
        if (detail.empty()) {
            detail = "action failed";
        }
        return {false, detail};
    }

    mode_.store(to, std::memory_order_release);
    ROS_INFO("[can_driver::OperationalCoordinator] %s -> %s",
             SystemOpModeName(current), SystemOpModeName(to));
    if (detail.empty()) {
        detail = std::string("-> ") + SystemOpModeName(to);
    }
    return {true, detail};
}

OperationalCoordinator::Result OperationalCoordinator::RequestInit(const std::string &device,
                                                                  bool loopback)
{
    return DoTransition(
        {SystemOpMode::Configured},
        SystemOpMode::Standby,
        [this, &device, loopback](std::string *detail) {
            if (!driverOps_.init_device) {
                if (detail) {
                    *detail = "init path not available";
                }
                return false;
            }
            const auto result = driverOps_.init_device(device, loopback);
            if (!result.ok) {
                if (detail) {
                    *detail = result.message;
                }
                return false;
            }
            if (driverOps_.hold_commands) {
                driverOps_.hold_commands();
            }
            if (driverOps_.arm_fresh_command_latch) {
                driverOps_.arm_fresh_command_latch();
            }
            if (detail && !result.message.empty()) {
                *detail = result.message;
            }
            return true;
        });
}

OperationalCoordinator::Result OperationalCoordinator::RequestEnable()
{
    return DoTransition(
        {SystemOpMode::Standby},
        SystemOpMode::Armed,
        [this](std::string *detail) {
            if (driverOps_.enable_healthy && !driverOps_.enable_healthy(detail)) {
                return false;
            }
            if (!driverOps_.enable_all) {
                if (detail) {
                    *detail = "enable path not available";
                }
                return false;
            }
            const auto result = driverOps_.enable_all();
            if (!result.ok) {
                if (detail) {
                    *detail = result.message;
                }
                return false;
            }
            if (driverOps_.hold_commands) {
                driverOps_.hold_commands();
            }
            if (driverOps_.arm_fresh_command_latch) {
                driverOps_.arm_fresh_command_latch();
            }
            if (detail && !result.message.empty()) {
                *detail = result.message;
            }
            return true;
        });
}

OperationalCoordinator::Result OperationalCoordinator::RequestDisable()
{
    return DoTransition({SystemOpMode::Standby, SystemOpMode::Armed, SystemOpMode::Running},
                        SystemOpMode::Standby,
                        [this](std::string *detail) {
                            if (driverOps_.hold_commands) {
                                driverOps_.hold_commands();
                            }
                            if (!driverOps_.disable_all) {
                                if (detail) {
                                    *detail = "disable path not available";
                                }
                                return false;
                            }
                            const auto result = driverOps_.disable_all();
                            if (!result.ok) {
                                if (detail) {
                                    *detail = result.message;
                                }
                                return false;
                            }
                            if (detail && !result.message.empty()) {
                                *detail = result.message;
                            }
                            return true;
                        });
}

OperationalCoordinator::Result OperationalCoordinator::RequestRelease()
{
    return DoTransition(
        {SystemOpMode::Armed},
        SystemOpMode::Running,
        [this](std::string *detail) {
            if (driverOps_.motion_healthy && !driverOps_.motion_healthy(detail)) {
                return false;
            }
            if (driverOps_.arm_fresh_command_latch) {
                driverOps_.arm_fresh_command_latch();
            }
            return true;
        });
}

OperationalCoordinator::Result OperationalCoordinator::RequestHalt()
{
    return DoTransition(
        {SystemOpMode::Running},
        SystemOpMode::Armed,
        [this](std::string *detail) {
            if (driverOps_.hold_commands) {
                driverOps_.hold_commands();
            }
            if (!driverOps_.halt_all) {
                if (detail) {
                    *detail = "halt path not available";
                }
                return false;
            }
            const auto result = driverOps_.halt_all();
            if (!result.ok) {
                if (detail) {
                    *detail = result.message;
                }
                return false;
            }
            if (detail && !result.message.empty()) {
                *detail = result.message;
            }
            return true;
        });
}

OperationalCoordinator::Result OperationalCoordinator::RequestRecover()
{
    return DoTransition(
        {SystemOpMode::Faulted},
        SystemOpMode::Standby,
        [this](std::string *detail) {
            mode_.store(SystemOpMode::Recovering, std::memory_order_release);
            if (!driverOps_.recover_all) {
                mode_.store(SystemOpMode::Faulted, std::memory_order_release);
                if (detail) {
                    *detail = "recover path not available";
                }
                return false;
            }
            const auto result = driverOps_.recover_all();
            if (!result.ok) {
                mode_.store(SystemOpMode::Faulted, std::memory_order_release);
                if (detail) {
                    *detail = result.message;
                }
                return false;
            }
            if (driverOps_.hold_commands) {
                driverOps_.hold_commands();
            }
            if (driverOps_.arm_fresh_command_latch) {
                driverOps_.arm_fresh_command_latch();
            }
            if (detail && !result.message.empty()) {
                *detail = result.message;
            }
            return true;
        });
}

OperationalCoordinator::Result OperationalCoordinator::RequestShutdown(bool force)
{
    return DoTransition(
        {SystemOpMode::Inactive, SystemOpMode::Configured, SystemOpMode::Standby,
         SystemOpMode::Armed, SystemOpMode::Running, SystemOpMode::Faulted,
         SystemOpMode::Recovering, SystemOpMode::ShuttingDown},
        SystemOpMode::Configured,
        [this, force](std::string *detail) {
            mode_.store(SystemOpMode::ShuttingDown, std::memory_order_release);
            if (!driverOps_.shutdown_all) {
                mode_.store(SystemOpMode::Configured, std::memory_order_release);
                if (detail) {
                    *detail = "shutdown path not available";
                }
                return false;
            }
            const auto result = driverOps_.shutdown_all(force);
            mode_.store(SystemOpMode::Configured, std::memory_order_release);
            if (!result.ok) {
                if (detail) {
                    *detail = result.message;
                }
                return false;
            }
            if (detail && !result.message.empty()) {
                *detail = result.message;
            }
            return true;
        });
}

void OperationalCoordinator::UpdateFromFeedback(bool unhealthy)
{
    if (!unhealthy) {
        return;
    }

    const auto current = mode_.load(std::memory_order_acquire);
    if (current != SystemOpMode::Armed && current != SystemOpMode::Running) {
        return;
    }

    SystemOpMode expected = current;
    if (mode_.compare_exchange_strong(expected,
                                      SystemOpMode::Faulted,
                                      std::memory_order_acq_rel)) {
        ROS_WARN("[can_driver::OperationalCoordinator] %s -> Faulted (auto)",
                 SystemOpModeName(current));
    }
}

} // namespace can_driver
