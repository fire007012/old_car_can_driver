#ifndef CAN_DRIVER_OPERATIONAL_COORDINATOR_HPP
#define CAN_DRIVER_OPERATIONAL_COORDINATOR_HPP

#include <atomic>
#include <functional>
#include <initializer_list>
#include <mutex>
#include <string>

namespace can_driver {

enum class SystemOpMode : unsigned char {
    Inactive,
    Configured,
    Standby,
    Armed,
    Running,
    Faulted,
    Recovering,
    ShuttingDown,
};

const char *SystemOpModeName(SystemOpMode mode);

class OperationalCoordinator {
public:
    struct Result {
        bool ok{false};
        std::string message;
    };

    struct DriverOps {
        std::function<Result(const std::string &, bool)> init_device;
        std::function<Result()> enable_all;
        std::function<Result()> disable_all;
        std::function<Result()> halt_all;
        std::function<Result()> recover_all;
        std::function<Result(bool)> shutdown_all;
        std::function<bool(std::string *)> enable_healthy;
        std::function<bool(std::string *)> motion_healthy;
        std::function<bool()> any_fault_active;
        std::function<void()> hold_commands;
        std::function<void()> arm_fresh_command_latch;
    };

    OperationalCoordinator() = default;
    explicit OperationalCoordinator(DriverOps driverOps);

    SystemOpMode mode() const
    {
        return mode_.load(std::memory_order_acquire);
    }

    void SetInactive();
    void SetConfigured();
    void SetFaulted();
    void SetDriverOps(DriverOps driverOps);

    Result RequestInit(const std::string &device, bool loopback);
    Result RequestEnable();
    Result RequestDisable();
    Result RequestRelease();
    Result RequestHalt();
    Result RequestRecover();
    Result RequestShutdown(bool force);
    void UpdateFromFeedback(bool unhealthy);

private:
    Result RevalidateArmedEnable();
    Result DoTransition(std::initializer_list<SystemOpMode> allowedFrom,
                        SystemOpMode to,
                        const std::function<bool(std::string *)> &action = nullptr);
    void ForceMode(SystemOpMode to, const char *reason);

    std::atomic<SystemOpMode> mode_{SystemOpMode::Inactive};
    mutable std::mutex transitionMutex_;
    DriverOps driverOps_;
};

} // namespace can_driver

#endif // CAN_DRIVER_OPERATIONAL_COORDINATOR_HPP
