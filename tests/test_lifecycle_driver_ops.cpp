#include <gtest/gtest.h>

#include "can_driver/AxisReadinessEvaluator.h"
#include "can_driver/IDeviceManager.h"
#include "can_driver/SharedDriverState.h"
#include "can_driver/lifecycle_driver_ops.hpp"

#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace {

class FakeProtocol : public CanProtocol {
public:
    bool setMode(MotorID, MotorMode) override { return true; }
    bool setVelocity(MotorID, int32_t) override { return true; }
    bool setAcceleration(MotorID, int32_t) override { return true; }
    bool setDeceleration(MotorID, int32_t) override { return true; }
    bool setPosition(MotorID, int32_t) override { return true; }
    bool quickSetPosition(MotorID, int32_t) override { return true; }

    bool Enable(MotorID motorId) override
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto id = static_cast<uint16_t>(motorId);
        ++enableCalls_[id];
        const bool result = lookupOr(enableResult_, id, true);
        if (result) {
            enabled_[id] = true;
        }
        return result;
    }

    bool Disable(MotorID motorId) override
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto id = static_cast<uint16_t>(motorId);
        ++disableCalls_[id];
        const bool result = lookupOr(disableResult_, id, true);
        if (!result) {
            return false;
        }
        enabled_[id] = false;
        return true;
    }

    bool Stop(MotorID motorId) override
    {
        std::lock_guard<std::mutex> lock(mutex_);
        ++stopCalls_[static_cast<uint16_t>(motorId)];
        return true;
    }

    bool ResetFault(MotorID motorId) override
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto id = static_cast<uint16_t>(motorId);
        ++resetFaultCalls_[id];
        const bool result = lookupOr(resetFaultResult_, id, true);
        if (result && lookupOr(clearFaultOnReset_, id, true)) {
            fault_[id] = false;
        }
        return result;
    }

    int64_t getPosition(MotorID) const override { return 0; }
    int16_t getCurrent(MotorID) const override { return 0; }
    int32_t getVelocity(MotorID) const override { return 0; }
    bool readPositionOffset(MotorID, int32_t*) override { return false; }
    void initializeMotorRefresh(const std::vector<MotorID> &) override {}

    bool isEnabled(MotorID motorId) const override
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return lookupOr(enabled_, static_cast<uint16_t>(motorId), false);
    }

    bool hasFault(MotorID motorId) const override
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return lookupOr(fault_, static_cast<uint16_t>(motorId), false);
    }

    void setEnableResult(uint16_t motorId, bool result)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        enableResult_[motorId] = result;
    }

    void setResetFaultBehavior(uint16_t motorId, bool result, bool clearFault)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        resetFaultResult_[motorId] = result;
        clearFaultOnReset_[motorId] = clearFault;
    }

    void setFault(uint16_t motorId, bool value)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        fault_[motorId] = value;
    }

    void setDisableResult(uint16_t motorId, bool result)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        disableResult_[motorId] = result;
    }

    int enableCalls(uint16_t motorId) const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return lookupOr(enableCalls_, motorId, 0);
    }

    int disableCalls(uint16_t motorId) const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return lookupOr(disableCalls_, motorId, 0);
    }

    int resetFaultCalls(uint16_t motorId) const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return lookupOr(resetFaultCalls_, motorId, 0);
    }

private:
    template <typename T>
    static T lookupOr(const std::map<uint16_t, T> &values, uint16_t key, T fallback)
    {
        const auto it = values.find(key);
        return it == values.end() ? fallback : it->second;
    }

    mutable std::mutex mutex_;
    std::map<uint16_t, bool> enableResult_;
    std::map<uint16_t, bool> disableResult_;
    std::map<uint16_t, bool> resetFaultResult_;
    std::map<uint16_t, bool> clearFaultOnReset_;
    std::map<uint16_t, bool> enabled_;
    std::map<uint16_t, bool> fault_;
    std::map<uint16_t, int> enableCalls_;
    std::map<uint16_t, int> disableCalls_;
    std::map<uint16_t, int> stopCalls_;
    std::map<uint16_t, int> resetFaultCalls_;
};

class FakeDeviceManager : public IDeviceManager {
public:
    FakeDeviceManager()
        : protocol_(std::make_shared<FakeProtocol>()),
          mutex_(std::make_shared<std::mutex>()),
          sharedState_(std::make_shared<can_driver::SharedDriverState>())
    {
    }

    bool ensureTransport(const std::string &, bool = false) override { return true; }
    bool ensureProtocol(const std::string &, CanType) override { return true; }

    bool initDevice(const std::string &device,
                    const std::vector<std::pair<CanType, MotorID>> &motors,
                    bool loopback = false) override
    {
        lastInitDevice_ = device;
        lastInitMotors_ = motors;
        lastInitLoopback_ = loopback;
        return initResult_;
    }

    void startRefresh(const std::string &, CanType, const std::vector<MotorID> &) override {}
    void setRefreshRateHz(double) override {}
    void setDeviceRefreshRateHz(const std::string &, double) override {}
    void setPpFastWriteEnabled(bool) override {}
    void setPpDefaultPositionVelocityRaw(int32_t) override {}
    void setPpPositionDefaultVelocityRaw(int32_t) override {}
    void setPpCspDefaultVelocityRaw(int32_t) override {}
    void shutdownDevice(const std::string &device) override
    {
        ++shutdownDeviceCalls_;
        lastShutdownDevice_ = device;
    }
    void shutdownAll() override { ++shutdownCalls_; }

    std::shared_ptr<CanProtocol> getProtocol(const std::string &, CanType) const override
    {
        return protocol_;
    }

    std::shared_ptr<std::mutex> getDeviceMutex(const std::string &) const override
    {
        return mutex_;
    }

    bool isDeviceReady(const std::string &) const override
    {
        return ready_;
    }

    std::size_t deviceCount() const override { return 1; }
    std::shared_ptr<can_driver::SharedDriverState> getSharedDriverState() const override
    {
        return sharedState_;
    }

    std::shared_ptr<FakeProtocol> protocol() const { return protocol_; }
    std::shared_ptr<can_driver::SharedDriverState> sharedState() const { return sharedState_; }

    void setReady(bool ready) { ready_ = ready; }
    void setInitResult(bool result) { initResult_ = result; }

    const std::string &lastInitDevice() const { return lastInitDevice_; }
    const std::vector<std::pair<CanType, MotorID>> &lastInitMotors() const { return lastInitMotors_; }
    bool lastInitLoopback() const { return lastInitLoopback_; }
    int shutdownDeviceCalls() const { return shutdownDeviceCalls_; }
    const std::string &lastShutdownDevice() const { return lastShutdownDevice_; }

private:
    std::shared_ptr<FakeProtocol> protocol_;
    std::shared_ptr<std::mutex> mutex_;
    std::shared_ptr<can_driver::SharedDriverState> sharedState_;
    bool ready_{true};
    bool initResult_{true};
    int shutdownDeviceCalls_{0};
    int shutdownCalls_{0};
    std::string lastInitDevice_;
    std::vector<std::pair<CanType, MotorID>> lastInitMotors_;
    bool lastInitLoopback_{false};
    std::string lastShutdownDevice_;
};

std::vector<MotorActionExecutor::Target> makeTargets()
{
    return {
        MotorActionExecutor::Target{"joint_a", "fake0", CanType::MT, static_cast<MotorID>(0x141)},
        MotorActionExecutor::Target{"joint_b", "fake0", CanType::MT, static_cast<MotorID>(0x142)},
        MotorActionExecutor::Target{"joint_c", "fake1", CanType::PP, static_cast<MotorID>(0x201)},
    };
}

TEST(AxisReadinessEvaluatorTest, PpAxisReportsReadyFactsBeforeAndAfterFirstCommand)
{
    const auto nowNs = can_driver::SharedDriverSteadyNowNs();
    const auto key = can_driver::MakeAxisKey("fake1", CanType::PP, static_cast<MotorID>(0x201));
    can_driver::AxisReadinessEvaluator evaluator;

    can_driver::SharedDriverState::AxisFeedbackState feedback;
    feedback.key = key;
    feedback.feedbackSeen = true;
    feedback.enabled = true;
    feedback.enabledValid = true;
    feedback.faultValid = true;
    feedback.lastRxSteadyNs = nowNs;

    can_driver::SharedDriverState::AxisCommandState command;
    command.key = key;

    can_driver::SharedDriverState::DeviceHealthState deviceHealth;
    deviceHealth.device = "fake1";
    deviceHealth.transportReady = true;

    const auto enabled = evaluator.Evaluate(
        feedback, &command, can_driver::AxisIntent::Enable, &deviceHealth, nowNs);
    EXPECT_TRUE(enabled.feedbackReady);
    EXPECT_TRUE(enabled.enabledReady);
    EXPECT_FALSE(enabled.commandValid);
    EXPECT_FALSE(enabled.modeExpected);
    EXPECT_TRUE(can_driver::AxisReadinessEvaluator::ReadyForEnable(enabled));
    EXPECT_TRUE(can_driver::AxisReadinessEvaluator::ReadyForRun(enabled));

    command.valid = true;
    command.desiredMode = CanProtocol::MotorMode::Position;
    command.desiredModeValid = true;
    feedback.mode = CanProtocol::MotorMode::Position;
    feedback.modeValid = true;

    const auto commanding = evaluator.Evaluate(
        feedback, &command, can_driver::AxisIntent::Enable, &deviceHealth, nowNs);
    EXPECT_TRUE(commanding.commandValid);
    EXPECT_TRUE(commanding.modeExpected);
    EXPECT_TRUE(commanding.modeMatched);
    EXPECT_TRUE(commanding.modeReady);
    EXPECT_TRUE(can_driver::AxisReadinessEvaluator::ReadyForEnable(commanding));
    EXPECT_TRUE(can_driver::AxisReadinessEvaluator::ReadyForRun(commanding));
}

TEST(AxisReadinessEvaluatorTest, PpAxisReportsDegradedAndFaultFactsOnUnhealthyPaths)
{
    const auto nowNs = can_driver::SharedDriverSteadyNowNs();
    const auto key = can_driver::MakeAxisKey("fake1", CanType::PP, static_cast<MotorID>(0x201));
    can_driver::AxisReadinessEvaluator evaluator;

    can_driver::SharedDriverState::AxisFeedbackState feedback;
    feedback.key = key;
    feedback.feedbackSeen = true;
    feedback.enabled = true;
    feedback.enabledValid = true;
    feedback.faultValid = true;
    feedback.lastRxSteadyNs = nowNs - 600000000LL;

    can_driver::SharedDriverState::AxisCommandState command;
    command.key = key;

    can_driver::SharedDriverState::DeviceHealthState deviceHealth;
    deviceHealth.device = "fake1";
    deviceHealth.transportReady = true;

    const auto degraded = evaluator.Evaluate(
        feedback, &command, can_driver::AxisIntent::Enable, &deviceHealth, nowNs);
    EXPECT_FALSE(degraded.feedbackReady);
    EXPECT_TRUE(degraded.enabledReady);
    EXPECT_FALSE(can_driver::AxisReadinessEvaluator::ReadyForEnable(degraded));
    EXPECT_FALSE(can_driver::AxisReadinessEvaluator::ReadyForRun(degraded));
    EXPECT_EQ(can_driver::AxisReadinessEvaluator::DescribeBlockReason(degraded),
              "Feedback degraded.");

    feedback.lastRxSteadyNs = nowNs;
    feedback.fault = true;
    const auto faultActive = evaluator.Evaluate(
        feedback, &command, can_driver::AxisIntent::Enable, &deviceHealth, nowNs);
    EXPECT_TRUE(faultActive.feedbackReady);
    EXPECT_FALSE(faultActive.faultCleared);
    EXPECT_FALSE(can_driver::AxisReadinessEvaluator::ReadyForEnable(faultActive));
    EXPECT_EQ(can_driver::AxisReadinessEvaluator::DescribeBlockReason(faultActive),
              "Fault still active.");
}

TEST(AxisReadinessEvaluatorTest, RecoverIntentRemainsPredicateOnlyWithoutLifecycleTracking)
{
    const auto nowNs = can_driver::SharedDriverSteadyNowNs();
    const auto key = can_driver::MakeAxisKey("fake1", CanType::PP, static_cast<MotorID>(0x201));
    can_driver::AxisReadinessEvaluator evaluator;

    can_driver::SharedDriverState::AxisFeedbackState feedback;
    feedback.key = key;
    feedback.feedbackSeen = true;
    feedback.enabled = true;
    feedback.enabledValid = true;
    feedback.faultValid = true;
    feedback.lastRxSteadyNs = nowNs;

    can_driver::SharedDriverState::AxisCommandState command;
    command.key = key;

    can_driver::SharedDriverState::DeviceHealthState deviceHealth;
    deviceHealth.device = "fake1";
    deviceHealth.transportReady = true;

    const auto firstRecovering = evaluator.Evaluate(
        feedback, &command, can_driver::AxisIntent::Recover, &deviceHealth, nowNs);
    EXPECT_TRUE(firstRecovering.axisReadyForEnable);
    EXPECT_FALSE(can_driver::AxisReadinessEvaluator::RecoverConfirmed(firstRecovering));
    EXPECT_TRUE(can_driver::AxisReadinessEvaluator::RecoverPending(firstRecovering));
    EXPECT_EQ(can_driver::AxisReadinessEvaluator::DescribeBlockReason(firstRecovering),
              "Axis still recovering.");

    const auto sameFrameRecovering = evaluator.Evaluate(
        feedback, &command, can_driver::AxisIntent::Recover, &deviceHealth, nowNs + 1000000LL);
    EXPECT_TRUE(sameFrameRecovering.axisReadyForEnable);
    EXPECT_FALSE(can_driver::AxisReadinessEvaluator::RecoverConfirmed(sameFrameRecovering));
    EXPECT_TRUE(can_driver::AxisReadinessEvaluator::RecoverPending(sameFrameRecovering));

    feedback.lastRxSteadyNs = nowNs + 20000000LL;
    feedback.fault = false;
    const auto nextRecovering = evaluator.Evaluate(
        feedback,
        &command,
        can_driver::AxisIntent::Recover,
        &deviceHealth,
        feedback.lastRxSteadyNs);
    EXPECT_TRUE(nextRecovering.axisReadyForEnable);
    EXPECT_FALSE(can_driver::AxisReadinessEvaluator::RecoverConfirmed(nextRecovering));
    EXPECT_TRUE(can_driver::AxisReadinessEvaluator::ReadyForEnable(nextRecovering));
    EXPECT_TRUE(can_driver::AxisReadinessEvaluator::RecoverPending(nextRecovering));
}

TEST(AxisReadinessEvaluatorTest, EnabledAxisStillRequiresSelectedModeToMatchBeforeRelease)
{
    const auto nowNs = can_driver::SharedDriverSteadyNowNs();
    const auto key = can_driver::MakeAxisKey("fake1", CanType::PP, static_cast<MotorID>(0x201));
    can_driver::AxisReadinessEvaluator evaluator;

    can_driver::SharedDriverState::AxisFeedbackState feedback;
    feedback.key = key;
    feedback.feedbackSeen = true;
    feedback.enabled = true;
    feedback.enabledValid = true;
    feedback.faultValid = true;
    feedback.mode = CanProtocol::MotorMode::Position;
    feedback.modeValid = true;
    feedback.lastRxSteadyNs = nowNs;

    can_driver::SharedDriverState::AxisCommandState command;
    command.key = key;
    command.desiredMode = CanProtocol::MotorMode::Velocity;
    command.desiredModeValid = true;
    command.valid = false;

    can_driver::SharedDriverState::DeviceHealthState deviceHealth;
    deviceHealth.device = "fake1";
    deviceHealth.transportReady = true;

    const auto enabledBlocked = evaluator.Evaluate(
        feedback, &command, can_driver::AxisIntent::Enable, &deviceHealth, nowNs);
    EXPECT_TRUE(enabledBlocked.feedbackReady);
    EXPECT_TRUE(enabledBlocked.enabledReady);
    EXPECT_FALSE(enabledBlocked.modeReady);
    EXPECT_TRUE(can_driver::AxisReadinessEvaluator::ReadyForEnable(enabledBlocked));
    EXPECT_FALSE(can_driver::AxisReadinessEvaluator::ReadyForRun(enabledBlocked));
    EXPECT_EQ(can_driver::AxisReadinessEvaluator::DescribeBlockReason(enabledBlocked),
              "Mode not ready.");

    feedback.mode = CanProtocol::MotorMode::Velocity;
    const auto enabledReady = evaluator.Evaluate(
        feedback, &command, can_driver::AxisIntent::Enable, &deviceHealth, nowNs);
    EXPECT_TRUE(enabledReady.modeReady);
    EXPECT_TRUE(can_driver::AxisReadinessEvaluator::ReadyForRun(enabledReady));
}

TEST(SharedDriverStateTest, CommandAndIntentUpdatesDoNotRewriteFeedbackFacts)
{
    can_driver::SharedDriverState state;
    const auto key = can_driver::MakeAxisKey("fake1", CanType::PP, static_cast<MotorID>(0x201));
    const auto lastRxNs = can_driver::SharedDriverSteadyNowNs();

    state.mutateAxisFeedback(
        key, [lastRxNs](can_driver::SharedDriverState::AxisFeedbackState *feedback) {
            feedback->feedbackSeen = true;
            feedback->enabled = true;
            feedback->enabledValid = true;
            feedback->faultValid = true;
            feedback->mode = CanProtocol::MotorMode::Position;
            feedback->modeValid = true;
            feedback->lastRxSteadyNs = lastRxNs;
            feedback->consecutiveTimeoutCount = 0;
        });

    can_driver::SharedDriverState::AxisFeedbackState before;
    ASSERT_TRUE(state.getAxisFeedback(key, &before));
    ASSERT_FALSE(before.degraded);

    state.mutateAxisCommand(
        key, [](can_driver::SharedDriverState::AxisCommandState *command) {
            command->valid = true;
            command->desiredMode = CanProtocol::MotorMode::Velocity;
            command->desiredModeValid = true;
            command->lastCommandSteadyNs = can_driver::SharedDriverSteadyNowNs();
        });
    state.setAxisIntent(key, can_driver::AxisIntent::Recover);

    can_driver::SharedDriverState::AxisFeedbackState after;
    ASSERT_TRUE(state.getAxisFeedback(key, &after));
    EXPECT_EQ(after.mode, CanProtocol::MotorMode::Position);
    EXPECT_EQ(after.lastRxSteadyNs, lastRxNs);
    EXPECT_TRUE(after.enabled);
    EXPECT_FALSE(after.degraded);
}

TEST(LifecycleDriverOpsTest, EnableAllRollsBackSucceededMotorsOnPartialFailure)
{
    auto deviceManager = std::make_shared<FakeDeviceManager>();
    MotorActionExecutor executor(deviceManager);
    can_driver::LifecycleDriverOps ops(deviceManager, &executor);
    ops.setTargets(makeTargets());

    deviceManager->protocol()->setEnableResult(0x141, true);
    deviceManager->protocol()->setEnableResult(0x142, false);

    const auto result = ops.enableAll();
    EXPECT_FALSE(result.ok);
    EXPECT_EQ(result.message, "Enable command rejected.");
    EXPECT_EQ(deviceManager->protocol()->disableCalls(0x141), 1);
    EXPECT_FALSE(deviceManager->protocol()->isEnabled(static_cast<MotorID>(0x141)));
}

TEST(LifecycleDriverOpsTest, EnableAllReportsRollbackFailureWhenDisableFails)
{
    auto deviceManager = std::make_shared<FakeDeviceManager>();
    MotorActionExecutor executor(deviceManager);
    can_driver::LifecycleDriverOps ops(deviceManager, &executor);
    ops.setTargets(makeTargets());

    deviceManager->protocol()->setEnableResult(0x141, true);
    deviceManager->protocol()->setEnableResult(0x142, false);
    deviceManager->protocol()->setDisableResult(0x141, false);

    const auto result = ops.enableAll();
    EXPECT_FALSE(result.ok);
    EXPECT_EQ(result.message, "Enable rollback failed after partial success.");
    EXPECT_EQ(deviceManager->protocol()->disableCalls(0x141), 1);
    EXPECT_TRUE(deviceManager->protocol()->isEnabled(static_cast<MotorID>(0x141)));
}

TEST(LifecycleDriverOpsTest, InitializeDeviceFiltersTargetsByRequestedBus)
{
    auto deviceManager = std::make_shared<FakeDeviceManager>();
    MotorActionExecutor executor(deviceManager);
    can_driver::LifecycleDriverOps ops(deviceManager, &executor);
    ops.setTargets(makeTargets());

    const auto result = ops.initializeDevice("fake1", true);
    ASSERT_TRUE(result.ok);
    EXPECT_EQ(deviceManager->lastInitDevice(), "fake1");
    ASSERT_EQ(deviceManager->lastInitMotors().size(), 1u);
    EXPECT_EQ(deviceManager->lastInitMotors().front().first, CanType::PP);
    EXPECT_EQ(static_cast<uint16_t>(deviceManager->lastInitMotors().front().second), 0x201u);
    EXPECT_TRUE(deviceManager->lastInitLoopback());
    EXPECT_EQ(deviceManager->protocol()->enableCalls(0x201), 1);
    EXPECT_EQ(deviceManager->protocol()->enableCalls(0x141), 0);
    EXPECT_EQ(deviceManager->protocol()->enableCalls(0x142), 0);
}

TEST(LifecycleDriverOpsTest, InitializeDeviceRollsBackPartialEnableFailure)
{
    auto deviceManager = std::make_shared<FakeDeviceManager>();
    MotorActionExecutor executor(deviceManager);
    can_driver::LifecycleDriverOps ops(deviceManager, &executor);
    ops.setTargets(makeTargets());

    deviceManager->protocol()->setEnableResult(0x141, true);
    deviceManager->protocol()->setEnableResult(0x142, false);

    const auto result = ops.initializeDevice("fake0", false);
    EXPECT_FALSE(result.ok);
    EXPECT_EQ(result.message, "Enable command rejected.");
    EXPECT_EQ(deviceManager->protocol()->enableCalls(0x141), 1);
    EXPECT_EQ(deviceManager->protocol()->enableCalls(0x142), 1);
    EXPECT_EQ(deviceManager->protocol()->disableCalls(0x141), 1);
    EXPECT_FALSE(deviceManager->protocol()->isEnabled(static_cast<MotorID>(0x141)));
}

TEST(LifecycleDriverOpsTest, ShutdownDeviceDelegatesToDeviceManager)
{
    auto deviceManager = std::make_shared<FakeDeviceManager>();
    MotorActionExecutor executor(deviceManager);
    can_driver::LifecycleDriverOps ops(deviceManager, &executor);

    const auto result = ops.shutdownDevice("fake1");

    EXPECT_TRUE(result.ok);
    EXPECT_EQ(deviceManager->shutdownDeviceCalls(), 1);
    EXPECT_EQ(deviceManager->lastShutdownDevice(), "fake1");
}

TEST(LifecycleDriverOpsTest, RecoverAllResetsFaultsBeforeReturningSuccess)
{
    auto deviceManager = std::make_shared<FakeDeviceManager>();
    MotorActionExecutor executor(deviceManager);
    can_driver::LifecycleDriverOps ops(deviceManager, &executor);
    ops.setTargets({
        MotorActionExecutor::Target{"joint_a", "fake0", CanType::MT, static_cast<MotorID>(0x141)},
        MotorActionExecutor::Target{"joint_b", "fake0", CanType::MT, static_cast<MotorID>(0x142)},
    });

    deviceManager->protocol()->setFault(0x141, true);
    deviceManager->protocol()->setFault(0x142, true);
    deviceManager->protocol()->setResetFaultBehavior(0x141, true, true);
    deviceManager->protocol()->setResetFaultBehavior(0x142, true, true);

    const auto result = ops.recoverAll();
    EXPECT_TRUE(result.ok);
    EXPECT_TRUE(result.message.empty());
    EXPECT_EQ(deviceManager->protocol()->resetFaultCalls(0x141), 1);
    EXPECT_EQ(deviceManager->protocol()->resetFaultCalls(0x142), 1);
    EXPECT_FALSE(ops.anyFaultActive());
}

TEST(LifecycleDriverOpsTest, EnableHealthyAcceptsFreshDisabledPpAxis)
{
    auto deviceManager = std::make_shared<FakeDeviceManager>();
    MotorActionExecutor executor(deviceManager);
    can_driver::LifecycleDriverOps ops(deviceManager, &executor);
    ops.setTargets({
        MotorActionExecutor::Target{"joint_c", "fake1", CanType::PP, static_cast<MotorID>(0x201)},
    });

    const auto nowNs = can_driver::SharedDriverSteadyNowNs();
    const auto axisKey = can_driver::MakeAxisKey("fake1", CanType::PP, static_cast<MotorID>(0x201));

    can_driver::SharedDriverState::AxisFeedbackState feedback;
    feedback.key = axisKey;
    feedback.feedbackSeen = true;
    feedback.enabled = false;
    feedback.enabledValid = true;
    feedback.fault = false;
    feedback.faultValid = true;
    feedback.lastRxSteadyNs = nowNs;
    deviceManager->sharedState()->mutateAxisFeedback(
        axisKey,
        [&](can_driver::SharedDriverState::AxisFeedbackState *state) {
            *state = feedback;
        });

    can_driver::SharedDriverState::DeviceHealthState deviceHealth;
    deviceHealth.device = "fake1";
    deviceHealth.transportReady = true;
    deviceManager->sharedState()->mutateDeviceHealth(
        "fake1",
        [&](can_driver::SharedDriverState::DeviceHealthState *state) {
            *state = deviceHealth;
        });

    std::string detail;
    EXPECT_TRUE(ops.enableHealthy(&detail));
    EXPECT_TRUE(detail.empty());
}

TEST(LifecycleDriverOpsTest, EnableHealthyRejectsFaultedPpAxisFromSharedState)
{
    auto deviceManager = std::make_shared<FakeDeviceManager>();
    MotorActionExecutor executor(deviceManager);
    can_driver::LifecycleDriverOps ops(deviceManager, &executor);
    ops.setTargets({
        MotorActionExecutor::Target{"joint_c", "fake1", CanType::PP, static_cast<MotorID>(0x201)},
    });

    const auto nowNs = can_driver::SharedDriverSteadyNowNs();
    const auto axisKey = can_driver::MakeAxisKey("fake1", CanType::PP, static_cast<MotorID>(0x201));

    can_driver::SharedDriverState::AxisFeedbackState feedback;
    feedback.key = axisKey;
    feedback.feedbackSeen = true;
    feedback.enabled = false;
    feedback.enabledValid = true;
    feedback.fault = true;
    feedback.faultValid = true;
    feedback.lastRxSteadyNs = nowNs;
    deviceManager->sharedState()->mutateAxisFeedback(
        axisKey,
        [&](can_driver::SharedDriverState::AxisFeedbackState *state) {
            *state = feedback;
        });

    can_driver::SharedDriverState::DeviceHealthState deviceHealth;
    deviceHealth.device = "fake1";
    deviceHealth.transportReady = true;
    deviceManager->sharedState()->mutateDeviceHealth(
        "fake1",
        [&](can_driver::SharedDriverState::DeviceHealthState *state) {
            *state = deviceHealth;
        });

    std::string detail;
    EXPECT_FALSE(ops.enableHealthy(&detail));
    EXPECT_EQ(detail, "Fault still active.");
}

TEST(LifecycleDriverOpsTest, MotionHealthyRequiresReadyDeviceAndClearedFaults)
{
    auto deviceManager = std::make_shared<FakeDeviceManager>();
    MotorActionExecutor executor(deviceManager);
    can_driver::LifecycleDriverOps ops(deviceManager, &executor);
    ops.setTargets({
        MotorActionExecutor::Target{"joint_a", "fake0", CanType::MT, static_cast<MotorID>(0x141)},
    });

    std::string detail;
    deviceManager->setReady(false);
    EXPECT_FALSE(ops.motionHealthy(&detail));
    EXPECT_EQ(detail, "CAN device not ready.");

    deviceManager->setReady(true);
    deviceManager->protocol()->setFault(0x141, true);
    EXPECT_FALSE(ops.motionHealthy(&detail));
    EXPECT_EQ(detail, "Fault still active.");

    deviceManager->protocol()->setFault(0x141, false);
    detail.clear();
    EXPECT_TRUE(ops.motionHealthy(&detail));
    EXPECT_TRUE(detail.empty());
}

TEST(LifecycleDriverOpsTest, MotionHealthyUsesSharedStateDegradationBeforeProtocolFallback)
{
    auto deviceManager = std::make_shared<FakeDeviceManager>();
    MotorActionExecutor executor(deviceManager);
    can_driver::LifecycleDriverOps ops(deviceManager, &executor);
    ops.setTargets({
        MotorActionExecutor::Target{"joint_a", "fake0", CanType::MT, static_cast<MotorID>(0x141)},
    });

    deviceManager->sharedState()->mutateDeviceHealth(
        "fake0",
        [](can_driver::SharedDriverState::DeviceHealthState *health) {
            health->transportReady = true;
        });
    deviceManager->sharedState()->mutateAxisFeedback(
        can_driver::MakeAxisKey("fake0", CanType::MT, static_cast<MotorID>(0x141)),
        [](can_driver::SharedDriverState::AxisFeedbackState *feedback) {
            feedback->feedbackSeen = true;
            feedback->enabled = true;
            feedback->enabledValid = true;
            feedback->faultValid = true;
            feedback->consecutiveTimeoutCount = 2;
        });

    std::string detail;
    EXPECT_FALSE(ops.motionHealthy(&detail));
    EXPECT_EQ(detail, "Feedback degraded.");
}

TEST(LifecycleDriverOpsTest, AnyFaultActiveUsesSharedStateFeedback)
{
    auto deviceManager = std::make_shared<FakeDeviceManager>();
    MotorActionExecutor executor(deviceManager);
    can_driver::LifecycleDriverOps ops(deviceManager, &executor);
    ops.setTargets({
        MotorActionExecutor::Target{"joint_a", "fake0", CanType::MT, static_cast<MotorID>(0x141)},
    });

    deviceManager->protocol()->setFault(0x141, false);
    deviceManager->sharedState()->mutateAxisFeedback(
        can_driver::MakeAxisKey("fake0", CanType::MT, static_cast<MotorID>(0x141)),
        [](can_driver::SharedDriverState::AxisFeedbackState *feedback) {
            feedback->feedbackSeen = true;
            feedback->fault = true;
            feedback->faultValid = true;
        });

    EXPECT_TRUE(ops.anyFaultActive());
}

TEST(LifecycleDriverOpsTest, MotionHealthyUsesAxisReadinessForPpModeMismatch)
{
    auto deviceManager = std::make_shared<FakeDeviceManager>();
    MotorActionExecutor executor(deviceManager);
    can_driver::LifecycleDriverOps ops(deviceManager, &executor);
    ops.setTargets({
        MotorActionExecutor::Target{"joint_c", "fake1", CanType::PP, static_cast<MotorID>(0x201)},
    });

    const auto key = can_driver::MakeAxisKey("fake1", CanType::PP, static_cast<MotorID>(0x201));
    deviceManager->sharedState()->mutateDeviceHealth(
        "fake1",
        [](can_driver::SharedDriverState::DeviceHealthState *health) {
            health->transportReady = true;
        });
    deviceManager->sharedState()->mutateAxisFeedback(
        key,
        [](can_driver::SharedDriverState::AxisFeedbackState *feedback) {
            feedback->feedbackSeen = true;
            feedback->enabled = true;
            feedback->enabledValid = true;
            feedback->faultValid = true;
            feedback->mode = CanProtocol::MotorMode::Velocity;
            feedback->modeValid = true;
            feedback->lastRxSteadyNs = can_driver::SharedDriverSteadyNowNs();
        });
    deviceManager->sharedState()->mutateAxisCommand(
        key,
        [](can_driver::SharedDriverState::AxisCommandState *command) {
            command->valid = true;
            command->desiredMode = CanProtocol::MotorMode::Position;
            command->desiredModeValid = true;
        });
    deviceManager->sharedState()->setAxisIntent(key, can_driver::AxisIntent::Enable);

    std::string detail;
    EXPECT_FALSE(ops.motionHealthy(&detail));
    EXPECT_EQ(detail, "Mode not ready.");
}

TEST(LifecycleDriverOpsTest, MotionHealthyRejectsStaleModeObservationAfterCommandModeChange)
{
    auto deviceManager = std::make_shared<FakeDeviceManager>();
    MotorActionExecutor executor(deviceManager);
    can_driver::LifecycleDriverOps ops(deviceManager, &executor);
    ops.setTargets({
        MotorActionExecutor::Target{"joint_c", "fake1", CanType::PP, static_cast<MotorID>(0x201)},
    });

    const auto key = can_driver::MakeAxisKey("fake1", CanType::PP, static_cast<MotorID>(0x201));
    const auto nowNs = can_driver::SharedDriverSteadyNowNs();
    deviceManager->sharedState()->mutateDeviceHealth(
        "fake1",
        [](can_driver::SharedDriverState::DeviceHealthState *health) {
            health->transportReady = true;
        });
    deviceManager->sharedState()->mutateAxisFeedback(
        key,
        [nowNs](can_driver::SharedDriverState::AxisFeedbackState *feedback) {
            feedback->feedbackSeen = true;
            feedback->enabled = true;
            feedback->enabledValid = true;
            feedback->faultValid = true;
            feedback->mode = CanProtocol::MotorMode::Position;
            feedback->modeValid = true;
            feedback->lastRxSteadyNs = nowNs;
        });
    deviceManager->sharedState()->mutateAxisCommand(
        key,
        [](can_driver::SharedDriverState::AxisCommandState *command) {
            command->valid = true;
            command->desiredMode = CanProtocol::MotorMode::Position;
            command->desiredModeValid = true;
        });
    deviceManager->sharedState()->setAxisIntent(key, can_driver::AxisIntent::Enable);

    std::string detail;
    EXPECT_TRUE(ops.motionHealthy(&detail));
    EXPECT_TRUE(detail.empty());

    deviceManager->sharedState()->mutateAxisCommand(
        key,
        [](can_driver::SharedDriverState::AxisCommandState *command) {
            command->valid = true;
            command->desiredMode = CanProtocol::MotorMode::Velocity;
            command->desiredModeValid = true;
        });

    EXPECT_FALSE(ops.motionHealthy(&detail));
    EXPECT_EQ(detail, "Mode not ready.");
}

TEST(LifecycleDriverOpsTest, MotionHealthyAcceptsFreshPpAxisBeforeFirstMotionCommand)
{
    auto deviceManager = std::make_shared<FakeDeviceManager>();
    MotorActionExecutor executor(deviceManager);
    can_driver::LifecycleDriverOps ops(deviceManager, &executor);
    ops.setTargets({
        MotorActionExecutor::Target{"joint_c", "fake1", CanType::PP, static_cast<MotorID>(0x201)},
    });

    deviceManager->sharedState()->mutateDeviceHealth(
        "fake1",
        [](can_driver::SharedDriverState::DeviceHealthState *health) {
            health->transportReady = true;
        });
    deviceManager->sharedState()->mutateAxisFeedback(
        can_driver::MakeAxisKey("fake1", CanType::PP, static_cast<MotorID>(0x201)),
        [](can_driver::SharedDriverState::AxisFeedbackState *feedback) {
            feedback->feedbackSeen = true;
            feedback->enabled = true;
            feedback->enabledValid = true;
            feedback->faultValid = true;
            feedback->lastRxSteadyNs = can_driver::SharedDriverSteadyNowNs();
        });

    std::string detail;
    EXPECT_TRUE(ops.motionHealthy(&detail));
    EXPECT_TRUE(detail.empty());
}

TEST(LifecycleDriverOpsTest, RecoverAllWaitsForAxisReadinessRecoveryConfirmation)
{
    auto deviceManager = std::make_shared<FakeDeviceManager>();
    MotorActionExecutor executor(deviceManager);
    can_driver::LifecycleDriverOps ops(deviceManager, &executor);
    ops.setTargets({
        MotorActionExecutor::Target{"joint_c", "fake1", CanType::PP, static_cast<MotorID>(0x201)},
    });

    const auto key = can_driver::MakeAxisKey("fake1", CanType::PP, static_cast<MotorID>(0x201));
    deviceManager->sharedState()->mutateDeviceHealth(
        "fake1",
        [](can_driver::SharedDriverState::DeviceHealthState *health) {
            health->transportReady = true;
        });
    deviceManager->sharedState()->setAxisIntent(key, can_driver::AxisIntent::Recover);
    deviceManager->sharedState()->mutateAxisFeedback(
        key,
        [](can_driver::SharedDriverState::AxisFeedbackState *feedback) {
            feedback->feedbackSeen = true;
            feedback->fault = true;
            feedback->faultValid = true;
            feedback->enabled = true;
            feedback->enabledValid = true;
            feedback->lastRxSteadyNs = can_driver::SharedDriverSteadyNowNs();
        });

    std::thread feedbackThread([sharedState = deviceManager->sharedState(), key]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
        sharedState->mutateAxisFeedback(
            key,
            [](can_driver::SharedDriverState::AxisFeedbackState *feedback) {
                feedback->fault = false;
                feedback->faultValid = true;
                feedback->enabled = true;
                feedback->enabledValid = true;
                feedback->lastRxSteadyNs = can_driver::SharedDriverSteadyNowNs();
            });
        std::this_thread::sleep_for(std::chrono::milliseconds(70));
        sharedState->mutateAxisFeedback(
            key,
            [](can_driver::SharedDriverState::AxisFeedbackState *feedback) {
                feedback->fault = false;
                feedback->faultValid = true;
                feedback->enabled = true;
                feedback->enabledValid = true;
                feedback->lastRxSteadyNs = can_driver::SharedDriverSteadyNowNs();
            });
    });

    const auto result = ops.recoverAll();
    feedbackThread.join();

    EXPECT_TRUE(result.ok);
    EXPECT_TRUE(result.message.empty());
}

} // namespace
