#ifndef CAN_DRIVER_AXIS_READINESS_EVALUATOR_H
#define CAN_DRIVER_AXIS_READINESS_EVALUATOR_H

#include "can_driver/SharedDriverState.h"

#include <algorithm>
#include <cstdint>
#include <string>

namespace can_driver {

struct AxisReadiness {
    SharedDriverState::AxisKey key;
    AxisIntent intent{AxisIntent::None};
    bool deviceReady{false};
    bool feedbackSeen{false};
    bool feedbackFresh{false};
    bool degraded{false};
    bool fault{false};
    bool faultKnown{false};
    bool faultCleared{false};
    bool enabled{false};
    bool enabledKnown{false};
    bool enabledReady{false};
    bool commandValid{false};
    bool modeExpected{false};
    bool modeKnown{true};
    bool modeMatched{true};
    bool modeReady{true};
    bool feedbackReady{false};
    bool axisReadyForEnable{false};
    bool axisReadyForRun{false};
    bool recoverConfirmed{false};
};

class AxisReadinessEvaluator {
public:
    struct Config {
        std::int64_t feedbackFreshnessTimeoutNs{500000000LL};
        std::uint32_t degradedTimeoutThreshold{kDefaultFeedbackDegradedTimeoutThreshold};
        std::uint32_t recoverConfirmCycles{2};
    };

    AxisReadinessEvaluator() = default;
    explicit AxisReadinessEvaluator(Config config)
        : config_(config)
    {
    }

    void setConfig(Config config)
    {
        config_ = config;
    }

    const Config &config() const
    {
        return config_;
    }

    AxisReadiness Evaluate(const SharedDriverState::AxisFeedbackState &feedback,
                           const SharedDriverState::AxisCommandState *command,
                           AxisIntent intent,
                           const SharedDriverState::DeviceHealthState *deviceHealth,
                           std::int64_t nowNs = SharedDriverSteadyNowNs()) const
    {
        AxisReadiness readiness = EvaluateBase(feedback, command, intent, deviceHealth, nowNs);
        readiness.recoverConfirmed = intent != AxisIntent::Recover && readiness.axisReadyForEnable;
        return readiness;
    }

    std::uint32_t recoverConfirmCycles() const
    {
        return std::max<std::uint32_t>(1u, config_.recoverConfirmCycles);
    }

    static bool ReadyForEnable(const AxisReadiness &readiness)
    {
        return readiness.axisReadyForEnable;
    }

    static bool ReadyForRun(const AxisReadiness &readiness)
    {
        return readiness.axisReadyForRun;
    }

    static bool RecoverConfirmed(const AxisReadiness &readiness)
    {
        return readiness.recoverConfirmed;
    }

    static bool MotionReady(const AxisReadiness &readiness)
    {
        return ReadyForRun(readiness);
    }

    static std::string DescribeBlockReason(const AxisReadiness &readiness)
    {
        if (!readiness.deviceReady || !readiness.feedbackSeen) {
            return "Feedback offline.";
        }
        if (readiness.degraded) {
            return "Feedback degraded.";
        }
        if (!readiness.feedbackFresh) {
            return "Feedback stale.";
        }
        if (!readiness.faultKnown) {
            return "Fault state unknown.";
        }
        if (!readiness.faultCleared) {
            return "Fault still active.";
        }
        if (!readiness.enabledKnown) {
            return "Enable state unknown.";
        }
        if (!readiness.enabledReady) {
            return "Axis not enabled.";
        }
        if (!readiness.modeReady) {
            return "Mode not ready.";
        }
        if (RecoverPending(readiness)) {
            return "Axis still recovering.";
        }
        return std::string();
    }

    static std::string DescribeMotionBlock(const AxisReadiness &readiness)
    {
        return DescribeBlockReason(readiness);
    }

    static bool RecoverPending(const AxisReadiness &readiness)
    {
        return readiness.intent == AxisIntent::Recover && !readiness.recoverConfirmed &&
               readiness.axisReadyForEnable;
    }

private:
    AxisReadiness EvaluateBase(const SharedDriverState::AxisFeedbackState &feedback,
                               const SharedDriverState::AxisCommandState *command,
                               AxisIntent intent,
                               const SharedDriverState::DeviceHealthState *deviceHealth,
                               std::int64_t nowNs) const
    {
        AxisReadiness readiness;
        readiness.key = feedback.key;
        readiness.intent = intent;
        readiness.deviceReady = (deviceHealth == nullptr) ? true : deviceHealth->transportReady;
        readiness.feedbackSeen = feedback.feedbackSeen;
        readiness.degraded = feedback.degraded ||
                             feedback.consecutiveTimeoutCount >=
                                 std::max<std::uint32_t>(1u,
                                                         config_.degradedTimeoutThreshold);
        readiness.fault = feedback.fault;
        readiness.faultKnown = feedback.faultValid;
        readiness.enabled = feedback.enabled;
        readiness.enabledKnown = feedback.enabledValid;
        readiness.commandValid = command != nullptr && command->valid;
        readiness.modeExpected = command != nullptr && command->desiredModeValid;

        readiness.feedbackFresh = readiness.feedbackSeen && feedback.lastRxSteadyNs > 0;
        if (readiness.feedbackFresh && config_.feedbackFreshnessTimeoutNs > 0 &&
            nowNs > feedback.lastRxSteadyNs) {
            readiness.feedbackFresh =
                (nowNs - feedback.lastRxSteadyNs) <= config_.feedbackFreshnessTimeoutNs;
        }

        if (readiness.modeExpected) {
            readiness.modeKnown = feedback.modeValid;
            readiness.modeMatched =
                feedback.modeValid && feedback.feedbackSeen && command != nullptr &&
                feedback.mode == command->desiredMode;
        }

        readiness.feedbackReady = readiness.deviceReady && readiness.feedbackSeen &&
                                  readiness.feedbackFresh && !readiness.degraded;
        readiness.faultCleared = readiness.faultKnown && !readiness.fault;
        readiness.enabledReady = readiness.enabledKnown && readiness.enabled;
        readiness.modeReady = !readiness.modeExpected || readiness.modeMatched;
        readiness.axisReadyForEnable = readiness.feedbackReady && readiness.faultCleared;
        readiness.axisReadyForRun =
            readiness.axisReadyForEnable && readiness.enabledReady && readiness.modeReady;
        return readiness;
    }

    Config config_{};
};

} // namespace can_driver

#endif // CAN_DRIVER_AXIS_READINESS_EVALUATOR_H
