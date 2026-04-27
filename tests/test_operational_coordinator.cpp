#include <gtest/gtest.h>

#include "can_driver/operational_coordinator.hpp"

namespace {

can_driver::OperationalCoordinator::DriverOps makeHappyOps()
{
    can_driver::OperationalCoordinator::DriverOps ops;
    ops.init_device = [](const std::string &, bool) {
        return can_driver::OperationalCoordinator::Result{true, "initialized"};
    };
    ops.enable_all = []() {
        return can_driver::OperationalCoordinator::Result{true, "enabled"};
    };
    ops.disable_all = []() {
        return can_driver::OperationalCoordinator::Result{true, "disabled"};
    };
    ops.halt_all = []() {
        return can_driver::OperationalCoordinator::Result{true, "halted"};
    };
    ops.recover_all = []() {
        return can_driver::OperationalCoordinator::Result{true, "recovered"};
    };
    ops.shutdown_all = [](bool) {
        return can_driver::OperationalCoordinator::Result{true, "shutdown"};
    };
    ops.enable_healthy = [](std::string *) {
        return true;
    };
    ops.motion_healthy = [](std::string *) {
        return true;
    };
    ops.any_fault_active = []() {
        return false;
    };
    ops.hold_commands = []() {};
    ops.arm_fresh_command_latch = []() {};
    return ops;
}

TEST(OperationalCoordinatorTest, TransitionMatrixFollowsLifecycleFlow)
{
    can_driver::OperationalCoordinator coordinator(makeHappyOps());

    coordinator.SetConfigured();
    EXPECT_EQ(coordinator.mode(), can_driver::SystemOpMode::Configured);

    auto result = coordinator.RequestInit("fake0", false);
    ASSERT_TRUE(result.ok);
    EXPECT_EQ(coordinator.mode(), can_driver::SystemOpMode::Standby);

    result = coordinator.RequestEnable();
    ASSERT_TRUE(result.ok);
    EXPECT_EQ(coordinator.mode(), can_driver::SystemOpMode::Armed);

    result = coordinator.RequestRelease();
    ASSERT_TRUE(result.ok);
    EXPECT_EQ(coordinator.mode(), can_driver::SystemOpMode::Running);

    result = coordinator.RequestHalt();
    ASSERT_TRUE(result.ok);
    EXPECT_EQ(coordinator.mode(), can_driver::SystemOpMode::Armed);

    result = coordinator.RequestDisable();
    ASSERT_TRUE(result.ok);
    EXPECT_EQ(coordinator.mode(), can_driver::SystemOpMode::Standby);

    result = coordinator.RequestShutdown(false);
    ASSERT_TRUE(result.ok);
    EXPECT_EQ(coordinator.mode(), can_driver::SystemOpMode::Configured);
}

TEST(OperationalCoordinatorTest, AutoFaultDowngradeFromRunning)
{
    can_driver::OperationalCoordinator coordinator(makeHappyOps());

    coordinator.SetConfigured();
    ASSERT_TRUE(coordinator.RequestInit("fake0", false).ok);
    ASSERT_TRUE(coordinator.RequestEnable().ok);
    ASSERT_TRUE(coordinator.RequestRelease().ok);
    ASSERT_EQ(coordinator.mode(), can_driver::SystemOpMode::Running);

    coordinator.UpdateFromFeedback(true);
    EXPECT_EQ(coordinator.mode(), can_driver::SystemOpMode::Faulted);
}

TEST(OperationalCoordinatorTest, RecoverFailureKeepsFaulted)
{
    auto ops = makeHappyOps();
    ops.recover_all = []() {
        return can_driver::OperationalCoordinator::Result{false, "recover failed"};
    };

    can_driver::OperationalCoordinator coordinator(ops);
    coordinator.SetFaulted();

    const auto result = coordinator.RequestRecover();
    EXPECT_FALSE(result.ok);
    EXPECT_EQ(result.message, "recover failed");
    EXPECT_EQ(coordinator.mode(), can_driver::SystemOpMode::Faulted);
}

TEST(OperationalCoordinatorTest, RecoverFromStandbyIsIdempotent)
{
    auto ops = makeHappyOps();
    ops.any_fault_active = []() {
        return true;
    };

    can_driver::OperationalCoordinator coordinator(ops);
    coordinator.SetConfigured();
    ASSERT_TRUE(coordinator.RequestInit("fake0", false).ok);
    ASSERT_EQ(coordinator.mode(), can_driver::SystemOpMode::Standby);

    const auto result = coordinator.RequestRecover();
    EXPECT_TRUE(result.ok);
    EXPECT_EQ(result.message, "already Standby");
    EXPECT_EQ(coordinator.mode(), can_driver::SystemOpMode::Standby);
}

TEST(OperationalCoordinatorTest, InitHoldsCommandsBeforeArmingFreshLatch)
{
    int callOrder = 0;
    int holdOrder = 0;
    int armOrder = 0;

    auto ops = makeHappyOps();
    ops.hold_commands = [&]() {
        holdOrder = ++callOrder;
    };
    ops.arm_fresh_command_latch = [&]() {
        armOrder = ++callOrder;
    };

    can_driver::OperationalCoordinator coordinator(ops);
    coordinator.SetConfigured();

    const auto result = coordinator.RequestInit("fake0", false);
    ASSERT_TRUE(result.ok);
    EXPECT_EQ(coordinator.mode(), can_driver::SystemOpMode::Standby);
    EXPECT_EQ(holdOrder, 1);
    EXPECT_EQ(armOrder, 2);
}

TEST(OperationalCoordinatorTest, EnableRejectsUnhealthyStandbyBeforeIssuingEnable)
{
    bool enableCalled = false;

    auto ops = makeHappyOps();
    ops.enable_healthy = [](std::string *detail) {
        if (detail) {
            *detail = "Fault still active.";
        }
        return false;
    };
    ops.enable_all = [&]() {
        enableCalled = true;
        return can_driver::OperationalCoordinator::Result{true, "enabled"};
    };

    can_driver::OperationalCoordinator coordinator(ops);
    coordinator.SetConfigured();
    ASSERT_TRUE(coordinator.RequestInit("fake0", false).ok);
    ASSERT_TRUE(coordinator.RequestDisable().ok);
    ASSERT_EQ(coordinator.mode(), can_driver::SystemOpMode::Standby);

    const auto result = coordinator.RequestEnable();
    EXPECT_FALSE(result.ok);
    EXPECT_EQ(result.message, "Fault still active.");
    EXPECT_FALSE(enableCalled);
    EXPECT_EQ(coordinator.mode(), can_driver::SystemOpMode::Standby);
}

} // namespace
