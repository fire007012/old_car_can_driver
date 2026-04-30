#ifndef CAN_DRIVER_SHARED_DRIVER_STATE_H
#define CAN_DRIVER_SHARED_DRIVER_STATE_H

#include "can_driver/CanProtocol.h"
#include "can_driver/CanType.h"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <map>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

namespace can_driver {

constexpr std::uint32_t kDefaultFeedbackDegradedTimeoutThreshold = 3u;

inline std::int64_t SharedDriverSteadyNowNs()
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

inline std::uint16_t NormalizeProtocolMotorId(MotorID motorId)
{
    // SharedDriverState 对外以“系统层 motor_id”为唯一轴标识；
    // 协议 node id 的低 8 位缩并只能在线路层使用，不能污染共享状态键。
    return toSystemMotorId(motorId);
}

enum class AxisIntent : std::uint8_t {
    None = 0,
    Disable,
    Enable,
    Hold,
    Run,
    Recover,
};

class SharedDriverState {
public:
    struct AxisKey {
        std::string device;
        CanType protocol{CanType::MT};
        std::uint16_t motorId{0};
    };

    struct AxisFeedbackState {
        AxisKey key;
        std::int64_t position{0};
        std::int32_t velocity{0};
        std::int32_t current{0};
        CanProtocol::MotorMode mode{CanProtocol::MotorMode::Position};
        bool positionValid{false};
        bool velocityValid{false};
        bool currentValid{false};
        bool modeValid{false};
        bool enabled{false};
        bool fault{false};
        bool enabledValid{false};
        bool faultValid{false};
        bool feedbackSeen{false};
        bool degraded{false};
        std::int64_t lastRxSteadyNs{0};
        std::int64_t lastValidStateSteadyNs{0};
        std::uint32_t consecutiveTimeoutCount{0};
    };

    struct AxisCommandState {
        AxisKey key;
        std::int64_t targetPosition{0};
        std::int32_t targetVelocity{0};
        std::int32_t targetCurrent{0};
        CanProtocol::MotorMode desiredMode{CanProtocol::MotorMode::Position};
        bool desiredModeValid{false};
        bool valid{false};
        std::int64_t lastCommandSteadyNs{0};
    };

    struct DeviceHealthState {
        std::string device;
        bool transportReady{false};
        std::uint64_t txBackpressure{0};
        std::uint64_t txLinkUnavailable{0};
        std::uint64_t txError{0};
        std::uint64_t rxError{0};
        std::int64_t lastTxLinkUnavailableSteadyNs{0};
        std::int64_t lastRxSteadyNs{0};
    };

    void registerAxis(const AxisKey &key)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        touchAxisLocked(key);
    }

    void registerAxis(const std::string &device, CanType protocol, MotorID motorId)
    {
        registerAxis(MakeAxisKey(device, protocol, motorId));
    }

    bool hasAxis(const AxisKey &key) const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return axisFeedback_.find(axisMapKey(key)) != axisFeedback_.end();
    }

    template <typename Fn>
    void mutateAxisFeedback(const AxisKey &key, Fn &&fn)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto mapKey = touchAxisLocked(key);
        auto &state = axisFeedback_[mapKey];
        fn(&state);
        recomputeAxisObservedLocked(&state);
        auto &deviceHealth = deviceHealth_[key.device];
        deviceHealth.device = key.device;
        if (state.lastRxSteadyNs > 0) {
            deviceHealth.lastRxSteadyNs =
                std::max(deviceHealth.lastRxSteadyNs, state.lastRxSteadyNs);
        }
    }

    bool getAxisFeedback(const AxisKey &key, AxisFeedbackState *out) const
    {
        if (!out) {
            return false;
        }

        std::lock_guard<std::mutex> lock(mutex_);
        const auto it = axisFeedback_.find(axisMapKey(key));
        if (it == axisFeedback_.end()) {
            return false;
        }
        *out = it->second;
        return true;
    }

    template <typename Fn>
    void mutateAxisCommand(const AxisKey &key, Fn &&fn)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto mapKey = touchAxisLocked(key);
        auto &state = axisCommands_[mapKey];
        fn(&state);
    }

    bool getAxisCommand(const AxisKey &key, AxisCommandState *out) const
    {
        if (!out) {
            return false;
        }

        std::lock_guard<std::mutex> lock(mutex_);
        const auto it = axisCommands_.find(axisMapKey(key));
        if (it == axisCommands_.end()) {
            return false;
        }
        *out = it->second;
        return true;
    }

    void setAxisIntent(const AxisKey &key, AxisIntent intent)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto mapKey = touchAxisLocked(key);
        axisIntents_[mapKey] = intent;
    }

    AxisIntent getAxisIntent(const AxisKey &key) const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto it = axisIntents_.find(axisMapKey(key));
        return (it == axisIntents_.end()) ? AxisIntent::None : it->second;
    }

    template <typename Fn>
    void mutateDeviceHealth(const std::string &device, Fn &&fn)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        auto &health = deviceHealth_[device];
        health.device = device;
        fn(&health);
    }

    bool getDeviceHealth(const std::string &device, DeviceHealthState *out) const
    {
        if (!out) {
            return false;
        }

        std::lock_guard<std::mutex> lock(mutex_);
        const auto it = deviceHealth_.find(device);
        if (it == deviceHealth_.end()) {
            return false;
        }
        *out = it->second;
        return true;
    }

private:
    static AxisKey MakeAxisKey(const std::string &device, CanType protocol, MotorID motorId)
    {
        AxisKey key;
        key.device = device;
        key.protocol = protocol;
        key.motorId = NormalizeProtocolMotorId(motorId);
        return key;
    }

    static const char *protocolName(CanType protocol)
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

    static std::string axisMapKey(const AxisKey &key)
    {
        return key.device + "|" + protocolName(key.protocol) + "|" +
               std::to_string(key.motorId);
    }

    std::string touchAxisLocked(const AxisKey &key)
    {
        const auto mapKey = axisMapKey(key);
        auto &feedback = axisFeedback_[mapKey];
        feedback.key = key;
        auto &command = axisCommands_[mapKey];
        command.key = key;
        axisIntents_.emplace(mapKey, AxisIntent::None);
        auto &health = deviceHealth_[key.device];
        health.device = key.device;
        return mapKey;
    }

    static void recomputeAxisObservedLocked(AxisFeedbackState *feedback)
    {
        if (!feedback) {
            return;
        }
        feedback->degraded = feedback->consecutiveTimeoutCount >=
                             kDefaultFeedbackDegradedTimeoutThreshold;
    }

    mutable std::mutex mutex_;
    std::map<std::string, AxisFeedbackState> axisFeedback_;
    std::map<std::string, AxisCommandState> axisCommands_;
    std::map<std::string, AxisIntent> axisIntents_;
    std::map<std::string, DeviceHealthState> deviceHealth_;
};

inline SharedDriverState::AxisKey MakeAxisKey(const std::string &device,
                                              CanType protocol,
                                              MotorID motorId)
{
    SharedDriverState::AxisKey key;
    key.device = device;
    key.protocol = protocol;
    key.motorId = NormalizeProtocolMotorId(motorId);
    return key;
}

} // namespace can_driver

#endif // CAN_DRIVER_SHARED_DRIVER_STATE_H
