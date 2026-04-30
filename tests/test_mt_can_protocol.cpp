#include <gtest/gtest.h>
#include <ros/time.h>

#include "can_driver/MtCan.h"
#include "can_driver/SharedDriverState.h"

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <vector>

namespace {

// Mock 传输层，隔离 MtCan 协议编解码逻辑。
class MockTransport : public CanTransport {
public:
    SendResult send(const Frame &frame) override
    {
        sentFrames.push_back(frame);
        return SendResult::Ok;
    }

    std::size_t addReceiveHandler(ReceiveHandler handler) override
    {
        receiveHandler = std::move(handler);
        return 1;
    }

    void removeReceiveHandler(std::size_t) override
    {
        receiveHandler = nullptr;
    }

    void simulateReceive(const Frame &frame) const
    {
        if (receiveHandler) {
            receiveHandler(frame);
        }
    }

    void clearSent()
    {
        sentFrames.clear();
    }

    std::vector<Frame> sentFrames;
    ReceiveHandler receiveHandler;
};

class MockTxDispatcher : public CanTxDispatcher {
public:
    explicit MockTxDispatcher(std::shared_ptr<MockTransport> transport)
        : transport(std::move(transport))
    {
    }

    void submit(const Request &request) override
    {
        requests.push_back(request);
        if (autoSend && transport) {
            const auto eventTime = std::chrono::steady_clock::now();
            const auto result = transport->send(request.frame);
            if (request.completion) {
                request.completion(true, result, eventTime);
            }
        }
    }

    std::shared_ptr<MockTransport> transport;
    std::vector<Request> requests;
    bool autoSend{true};
};

class MtCanTest : public ::testing::Test {
protected:
    static void SetUpTestSuite()
    {
        ros::Time::init();
    }

    MtCanTest()
        : transport(std::make_shared<MockTransport>())
        , txDispatcher(std::make_shared<MockTxDispatcher>(transport))
        , sharedState(std::make_shared<can_driver::SharedDriverState>())
        , mt(transport, txDispatcher, sharedState, "can0")
    {
    }

    std::shared_ptr<MockTransport> transport;
    std::shared_ptr<MockTxDispatcher> txDispatcher;
    std::shared_ptr<can_driver::SharedDriverState> sharedState;
    MtCan mt;
};

} // namespace

class MtCanTestAccessor {
public:
    static void ageAllPendingRequests(MtCan &mt, std::chrono::milliseconds age)
    {
        const auto now = std::chrono::steady_clock::now();
        std::lock_guard<std::mutex> lock(mt.pendingReadMutex_);
        for (auto &entry : mt.pendingReadRequests_) {
            entry.second.inFlight = true;
            entry.second.lastSent = now - age;
        }
    }

    static void expireAllPendingBackoff(MtCan &mt)
    {
        std::lock_guard<std::mutex> lock(mt.pendingReadMutex_);
        for (auto &entry : mt.pendingReadRequests_) {
            entry.second.nextEligibleSend = std::chrono::steady_clock::time_point {};
        }
    }

    static std::size_t consecutiveTimeouts(MtCan &mt, uint8_t motorId, uint8_t command)
    {
        std::lock_guard<std::mutex> lock(mt.pendingReadMutex_);
        const auto it = mt.pendingReadRequests_.find(MtCan::pendingReadKey(motorId, command));
        return (it == mt.pendingReadRequests_.end()) ? 0u : it->second.consecutiveTimeouts;
    }

    static bool isQueued(MtCan &mt, uint8_t motorId, uint8_t command)
    {
        std::lock_guard<std::mutex> lock(mt.pendingReadMutex_);
        const auto it = mt.pendingReadRequests_.find(MtCan::pendingReadKey(motorId, command));
        return (it == mt.pendingReadRequests_.end()) ? false : it->second.queued;
    }

    static bool isInFlight(MtCan &mt, uint8_t motorId, uint8_t command)
    {
        std::lock_guard<std::mutex> lock(mt.pendingReadMutex_);
        const auto it = mt.pendingReadRequests_.find(MtCan::pendingReadKey(motorId, command));
        return (it == mt.pendingReadRequests_.end()) ? false : it->second.inFlight;
    }

};

TEST_F(MtCanTest, SetVelocityEncodesExpectedFrame)
{
    constexpr MotorID kMotorId = static_cast<MotorID>(0x01);
    constexpr int32_t kVelocity = -123456;

    ASSERT_TRUE(mt.setVelocity(kMotorId, kVelocity));
    ASSERT_EQ(transport->sentFrames.size(), 1u);

    const auto &frame = transport->sentFrames[0];
    // MT 速度命令使用 0xA2，速度值小端写入 data[4..7]。
    EXPECT_EQ(frame.id, 0x141u);
    EXPECT_FALSE(frame.isExtended);
    EXPECT_FALSE(frame.isRemoteRequest);
    EXPECT_EQ(frame.dlc, 8u);
    EXPECT_EQ(frame.data[0], 0xA2);
    EXPECT_EQ(frame.data[4], static_cast<uint8_t>(kVelocity & 0xFF));
    EXPECT_EQ(frame.data[5], static_cast<uint8_t>((kVelocity >> 8) & 0xFF));
    EXPECT_EQ(frame.data[6], static_cast<uint8_t>((kVelocity >> 16) & 0xFF));
    EXPECT_EQ(frame.data[7], static_cast<uint8_t>((kVelocity >> 24) & 0xFF));
}

TEST_F(MtCanTest, SetPositionEncodesExpectedFrame)
{
    constexpr MotorID kMotorId = static_cast<MotorID>(0x02);
    constexpr int32_t kVelocity = 0x1234;
    constexpr int32_t kPosition = 0x01020304;

    ASSERT_TRUE(mt.setVelocity(kMotorId, kVelocity));
    transport->clearSent();
    ASSERT_TRUE(mt.setPosition(kMotorId, kPosition));
    ASSERT_EQ(transport->sentFrames.size(), 1u);

    const auto &frame = transport->sentFrames[0];
    // MT 位置命令 0xA4：携带速度上限（data[2..3]）和目标位置（data[4..7]）。
    EXPECT_EQ(frame.id, 0x142u);
    EXPECT_EQ(frame.dlc, 8u);
    EXPECT_EQ(frame.data[0], 0xA4);
    // 当前实现会把 setVelocity 的 0.01 dps/LSB 换算到 0xA4 所需的 1 dps/LSB。
    // 0x1234 (4660) -> round(46.6) = 47 -> 0x002F。
    EXPECT_EQ(frame.data[2], 47u);
    EXPECT_EQ(frame.data[3], 0u);
    EXPECT_EQ(frame.data[4], static_cast<uint8_t>(kPosition & 0xFF));
    EXPECT_EQ(frame.data[5], static_cast<uint8_t>((kPosition >> 8) & 0xFF));
    EXPECT_EQ(frame.data[6], static_cast<uint8_t>((kPosition >> 16) & 0xFF));
    EXPECT_EQ(frame.data[7], static_cast<uint8_t>((kPosition >> 24) & 0xFF));
}

TEST_F(MtCanTest, SetPositionWithoutVelocityUsesDefaultSpeed)
{
    constexpr MotorID kMotorId = static_cast<MotorID>(0x02);
    constexpr int32_t kPosition = 0x01020304;

    ASSERT_TRUE(mt.setPosition(kMotorId, kPosition));
    ASSERT_EQ(transport->sentFrames.size(), 1u);

    const auto &frame = transport->sentFrames[0];
    EXPECT_EQ(frame.data[0], 0xA4);
    EXPECT_EQ(frame.data[2], 100u);
    EXPECT_EQ(frame.data[3], 0u);
}

TEST_F(MtCanTest, FullMtMotorIdUsesMtNodeOffset)
{
    constexpr MotorID kMotorId = static_cast<MotorID>(0x143);

    ASSERT_TRUE(mt.setVelocity(kMotorId, 321));
    ASSERT_EQ(transport->sentFrames.size(), 1u);

    const auto &frame = transport->sentFrames[0];
    EXPECT_EQ(frame.id, 0x143u);
    EXPECT_EQ(frame.data[0], 0xA2);
}

TEST_F(MtCanTest, WritesRouteThroughUnifiedTxDispatcher)
{
    ASSERT_TRUE(mt.setVelocity(static_cast<MotorID>(0x01), 123));
    ASSERT_EQ(txDispatcher->requests.size(), 1u);
    EXPECT_EQ(txDispatcher->requests[0].category, CanTxDispatcher::Category::Control);
    EXPECT_STREQ(txDispatcher->requests[0].source, "MtCan::sendFrame");
}

TEST_F(MtCanTest, CommandsPopulateSharedStateIntentAndTargets)
{
    constexpr MotorID kMotorId = static_cast<MotorID>(0x01);
    const auto axisKey = can_driver::MakeAxisKey("can0", CanType::MT, kMotorId);

    ASSERT_TRUE(mt.setVelocity(kMotorId, 4321));
    ASSERT_TRUE(mt.Enable(kMotorId));

    can_driver::SharedDriverState::AxisCommandState command;
    ASSERT_TRUE(sharedState->getAxisCommand(axisKey, &command));
    EXPECT_EQ(command.targetPosition, 0);
    EXPECT_EQ(command.targetVelocity, 4321);
    EXPECT_EQ(command.desiredMode, CanProtocol::MotorMode::Velocity);
    EXPECT_TRUE(command.desiredModeValid);
    EXPECT_TRUE(command.valid);
    EXPECT_EQ(sharedState->getAxisIntent(axisKey), can_driver::AxisIntent::Enable);
}

TEST_F(MtCanTest, SetModeUpdatesDesiredModeWithoutPretendingMotionCommandExists)
{
    constexpr MotorID kMotorId = static_cast<MotorID>(0x01);
    const auto axisKey = can_driver::MakeAxisKey("can0", CanType::MT, kMotorId);

    ASSERT_TRUE(mt.setVelocity(kMotorId, 4321));
    ASSERT_TRUE(mt.setMode(kMotorId, CanProtocol::MotorMode::Position));

    can_driver::SharedDriverState::AxisCommandState command;
    ASSERT_TRUE(sharedState->getAxisCommand(axisKey, &command));
    EXPECT_EQ(command.desiredMode, CanProtocol::MotorMode::Position);
    EXPECT_TRUE(command.desiredModeValid);
    EXPECT_FALSE(command.valid);
    EXPECT_EQ(command.targetPosition, 0);
    EXPECT_EQ(command.targetVelocity, 0);
    EXPECT_EQ(command.lastCommandSteadyNs, 0);
}

TEST_F(MtCanTest, IssueRefreshQueryMapsEnumsToExpectedCommands)
{
    constexpr MotorID kMotorId = static_cast<MotorID>(0x01);

    EXPECT_TRUE(mt.issueRefreshQuery(kMotorId, MtCan::RefreshQuery::State));
    EXPECT_TRUE(mt.issueRefreshQuery(kMotorId, MtCan::RefreshQuery::MultiTurnAngle));
    EXPECT_TRUE(mt.issueRefreshQuery(kMotorId, MtCan::RefreshQuery::Error));

    ASSERT_EQ(txDispatcher->requests.size(), 3u);
    ASSERT_EQ(transport->sentFrames.size(), 3u);
    EXPECT_EQ(txDispatcher->requests[0].category, CanTxDispatcher::Category::Query);
    EXPECT_STREQ(txDispatcher->requests[0].source, "MtCan::requestState");
    EXPECT_EQ(txDispatcher->requests[1].category, CanTxDispatcher::Category::Query);
    EXPECT_STREQ(txDispatcher->requests[1].source, "MtCan::requestMultiTurnAngle");
    EXPECT_EQ(txDispatcher->requests[2].category, CanTxDispatcher::Category::Query);
    EXPECT_STREQ(txDispatcher->requests[2].source, "MtCan::requestError");
    EXPECT_EQ(transport->sentFrames[0].data[0], 0x9Cu);
    EXPECT_EQ(transport->sentFrames[1].data[0], 0x92u);
    EXPECT_EQ(transport->sentFrames[2].data[0], 0x9Au);
}

TEST_F(MtCanTest, HandleResponseParsesStateFrame)
{
    // 响应 CAN ID=0x240+nodeId，nodeId=1 -> CAN ID 0x241。
    constexpr MotorID kResponseNodeId = static_cast<MotorID>(0x01);

    // 0x9C 状态反馈帧：电流/速度等状态由协议层解码后更新缓存。
    CanTransport::Frame frame {};
    frame.id = 0x241;
    frame.dlc = 8;
    frame.isExtended = false;
    frame.isRemoteRequest = false;
    frame.data[0] = 0x9C;
    frame.data[2] = 0x7B; // current raw low byte (123)
    frame.data[3] = 0x00; // current raw high byte
    frame.data[4] = 0x58; // velocity raw low byte (600)
    frame.data[5] = 0x02; // velocity raw high byte

    transport->simulateReceive(frame);

    EXPECT_EQ(mt.getCurrent(kResponseNodeId), 123);
    EXPECT_EQ(mt.getVelocity(kResponseNodeId), 600);
}

TEST_F(MtCanTest, ResponsesUpdateSharedFeedbackFreshness)
{
    constexpr MotorID kMotorId = static_cast<MotorID>(0x01);
    const auto axisKey = can_driver::MakeAxisKey("can0", CanType::MT, kMotorId);

    CanTransport::Frame frame {};
    frame.id = 0x241;
    frame.dlc = 8;
    frame.isExtended = false;
    frame.isRemoteRequest = false;
    frame.data[0] = 0x9C;
    frame.data[2] = 0x7B;
    frame.data[3] = 0x00;
    frame.data[4] = 0x58;
    frame.data[5] = 0x02;

    transport->simulateReceive(frame);

    can_driver::SharedDriverState::AxisFeedbackState feedback;
    ASSERT_TRUE(sharedState->getAxisFeedback(axisKey, &feedback));
    EXPECT_TRUE(feedback.feedbackSeen);
    EXPECT_TRUE(feedback.currentValid);
    EXPECT_TRUE(feedback.velocityValid);
    EXPECT_EQ(feedback.current, 123);
    EXPECT_EQ(feedback.velocity, 600);
    EXPECT_EQ(feedback.consecutiveTimeoutCount, 0u);
    EXPECT_GT(feedback.lastRxSteadyNs, 0);
}

TEST_F(MtCanTest, HandleResponseIgnoresExtendedFrame)
{
    constexpr MotorID kResponseNodeId = static_cast<MotorID>(0x01);

    CanTransport::Frame frame {};
    frame.id = 0x241;
    frame.dlc = 8;
    frame.isExtended = true;
    frame.data[0] = 0x9C;
    frame.data[2] = 0x7B;
    frame.data[3] = 0x00;
    frame.data[4] = 0x58;
    frame.data[5] = 0x02;

    transport->simulateReceive(frame);

    EXPECT_EQ(mt.getCurrent(kResponseNodeId), 0);
    EXPECT_EQ(mt.getVelocity(kResponseNodeId), 0);
}

TEST_F(MtCanTest, GetPositionParsesProtocolMultiTurnAngleDataBytes)
{
    constexpr MotorID kResponseNodeId = static_cast<MotorID>(0x01);

    // 0x92 多圈角度按协议从 DATA[4..7] 读取，单位 0.01 deg/LSB。
    // 协议示例 0x00008CA0 = 36000 -> 360 deg。
    CanTransport::Frame frame {};
    frame.id = 0x241;
    frame.dlc = 8;
    frame.isExtended = false;
    frame.isRemoteRequest = false;
    frame.data[0] = 0x92;
    frame.data[1] = 0x00;
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = 0xA0;
    frame.data[5] = 0x8C;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;

    transport->simulateReceive(frame);

    EXPECT_EQ(mt.getPosition(kResponseNodeId), 36000LL);

    // 负值也应按 int32 符号扩展。
    frame.data[4] = 0x60;
    frame.data[5] = 0x79;
    frame.data[6] = 0xFF;
    frame.data[7] = 0xFF;

    transport->simulateReceive(frame);

    EXPECT_EQ(mt.getPosition(kResponseNodeId), -34464LL);
}

TEST_F(MtCanTest, GetPositionWithoutCacheReturnsZeroWithoutSendingReadRequest)
{
    EXPECT_EQ(mt.getPosition(static_cast<MotorID>(0x01)), 0);
    EXPECT_TRUE(transport->sentFrames.empty());
}

TEST_F(MtCanTest, IssueRefreshQueryDoesNotResendSameReadWhileRequestIsInFlight)
{
    EXPECT_TRUE(mt.issueRefreshQuery(static_cast<MotorID>(0x01), MtCan::RefreshQuery::State));
    ASSERT_EQ(transport->sentFrames.size(), 1u);

    EXPECT_FALSE(mt.issueRefreshQuery(static_cast<MotorID>(0x01), MtCan::RefreshQuery::State));
    EXPECT_EQ(transport->sentFrames.size(), 1u);
}

TEST_F(MtCanTest, QueuedReadRequestDoesNotStartTimeoutBeforeActualSend)
{
    txDispatcher->autoSend = false;

    EXPECT_TRUE(mt.issueRefreshQuery(static_cast<MotorID>(0x01), MtCan::RefreshQuery::State));
    ASSERT_EQ(txDispatcher->requests.size(), 1u);
    EXPECT_TRUE(MtCanTestAccessor::isQueued(mt, 0x01, 0x9C));
    EXPECT_FALSE(MtCanTestAccessor::isInFlight(mt, 0x01, 0x9C));

    MtCanTestAccessor::ageAllPendingRequests(mt, std::chrono::milliseconds(500));
    EXPECT_FALSE(mt.issueRefreshQuery(static_cast<MotorID>(0x01), MtCan::RefreshQuery::State));

    EXPECT_EQ(txDispatcher->requests.size(), 1u);
    EXPECT_EQ(MtCanTestAccessor::consecutiveTimeouts(mt, 0x01, 0x9C), 0u);
    EXPECT_TRUE(MtCanTestAccessor::isQueued(mt, 0x01, 0x9C));
}

TEST_F(MtCanTest, IssueRefreshQueryBacksOffAfterRepeatedReadTimeouts)
{
    EXPECT_TRUE(mt.issueRefreshQuery(static_cast<MotorID>(0x01), MtCan::RefreshQuery::State));
    ASSERT_EQ(transport->sentFrames.size(), 1u);

    MtCanTestAccessor::ageAllPendingRequests(mt, std::chrono::milliseconds(260));
    EXPECT_FALSE(mt.issueRefreshQuery(static_cast<MotorID>(0x01), MtCan::RefreshQuery::State));
    EXPECT_EQ(transport->sentFrames.size(), 1u);
    EXPECT_EQ(MtCanTestAccessor::consecutiveTimeouts(mt, 0x01, 0x9C), 1u);

    MtCanTestAccessor::expireAllPendingBackoff(mt);
    EXPECT_TRUE(mt.issueRefreshQuery(static_cast<MotorID>(0x01), MtCan::RefreshQuery::State));
    EXPECT_EQ(transport->sentFrames.size(), 2u);

    can_driver::SharedDriverState::AxisFeedbackState feedback;
    ASSERT_TRUE(
        sharedState->getAxisFeedback(can_driver::MakeAxisKey("can0", CanType::MT, static_cast<MotorID>(0x01)),
                                     &feedback));
    EXPECT_EQ(feedback.consecutiveTimeoutCount, 1u);
    EXPECT_FALSE(feedback.degraded);
}

TEST_F(MtCanTest, FreshFeedbackClearsSharedTimeoutDegradedState)
{
    constexpr MotorID kMotorId = static_cast<MotorID>(0x01);

    mt.issueRefreshQuery(kMotorId, MtCan::RefreshQuery::State);
    ASSERT_EQ(transport->sentFrames.size(), 1u);

    MtCanTestAccessor::ageAllPendingRequests(mt, std::chrono::milliseconds(260));
    mt.issueRefreshQuery(kMotorId, MtCan::RefreshQuery::State);

    can_driver::SharedDriverState::AxisFeedbackState feedback;
    ASSERT_TRUE(sharedState->getAxisFeedback(
        can_driver::MakeAxisKey("can0", CanType::MT, kMotorId), &feedback));
    ASSERT_EQ(feedback.consecutiveTimeoutCount, 1u);
    ASSERT_FALSE(feedback.degraded);

    CanTransport::Frame frame;
    frame.id = 0x241;
    frame.dlc = 8;
    frame.isExtended = false;
    frame.data.fill(0);
    frame.data[0] = 0x9C;
    frame.data[4] = 0x34;
    frame.data[5] = 0x12;
    transport->simulateReceive(frame);

    ASSERT_TRUE(sharedState->getAxisFeedback(
        can_driver::MakeAxisKey("can0", CanType::MT, kMotorId), &feedback));
    EXPECT_EQ(feedback.consecutiveTimeoutCount, 0u);
    EXPECT_FALSE(feedback.degraded);
    EXPECT_TRUE(feedback.feedbackSeen);
}

TEST_F(MtCanTest, SlowRefreshRateDoesNotPrematurelyTimeoutReadRequests)
{
    mt.setRefreshRateHz(5.0);
    mt.initializeMotorRefresh({static_cast<MotorID>(0x01), static_cast<MotorID>(0x02), static_cast<MotorID>(0x03)});
    transport->clearSent();

    mt.issueRefreshQuery(static_cast<MotorID>(0x01), MtCan::RefreshQuery::State);
    ASSERT_EQ(transport->sentFrames.size(), 1u);

    MtCanTestAccessor::ageAllPendingRequests(mt, std::chrono::milliseconds(700));
    mt.issueRefreshQuery(static_cast<MotorID>(0x01), MtCan::RefreshQuery::State);
    EXPECT_EQ(transport->sentFrames.size(), 1u);
    EXPECT_EQ(MtCanTestAccessor::consecutiveTimeouts(mt, 0x01, 0x9C), 0u);

    MtCanTestAccessor::ageAllPendingRequests(mt, std::chrono::milliseconds(1300));
    mt.issueRefreshQuery(static_cast<MotorID>(0x01), MtCan::RefreshQuery::State);
    EXPECT_EQ(transport->sentFrames.size(), 1u);
    EXPECT_EQ(MtCanTestAccessor::consecutiveTimeouts(mt, 0x01, 0x9C), 1u);
}

TEST_F(MtCanTest, InitializeMotorRefreshBroadcastsConfiguredCommunicationTimeout)
{
    mt.setCommunicationTimeoutOnInit(0);

    mt.initializeMotorRefresh({static_cast<MotorID>(0x01), static_cast<MotorID>(0x02)});

    ASSERT_EQ(transport->sentFrames.size(), 2u);
    EXPECT_EQ(transport->sentFrames[0].id, 0x141u);
    EXPECT_EQ(transport->sentFrames[0].data[0], 0xB3u);
    EXPECT_EQ(transport->sentFrames[0].data[4], 0u);
    EXPECT_EQ(transport->sentFrames[0].data[5], 0u);
    EXPECT_EQ(transport->sentFrames[0].data[6], 0u);
    EXPECT_EQ(transport->sentFrames[0].data[7], 0u);
    EXPECT_EQ(transport->sentFrames[1].id, 0x142u);
    EXPECT_EQ(transport->sentFrames[1].data[0], 0xB3u);
}

TEST_F(MtCanTest, EnableDisableAndFaultStateAreObservable)
{
    constexpr MotorID kMotorId = static_cast<MotorID>(0x01);
    const auto axisKey = can_driver::MakeAxisKey("can0", CanType::MT, kMotorId);

    EXPECT_TRUE(mt.Enable(kMotorId));
    EXPECT_TRUE(mt.isEnabled(kMotorId));

    can_driver::SharedDriverState::AxisFeedbackState feedback;
    ASSERT_TRUE(sharedState->getAxisFeedback(axisKey, &feedback));
    EXPECT_TRUE(feedback.feedbackSeen);
    EXPECT_TRUE(feedback.enabledValid);
    EXPECT_TRUE(feedback.enabled);

    EXPECT_TRUE(mt.Disable(kMotorId));
    EXPECT_FALSE(mt.isEnabled(kMotorId));
    ASSERT_TRUE(sharedState->getAxisFeedback(axisKey, &feedback));
    EXPECT_TRUE(feedback.feedbackSeen);
    EXPECT_TRUE(feedback.enabledValid);
    EXPECT_FALSE(feedback.enabled);
    ASSERT_FALSE(transport->sentFrames.empty());
    EXPECT_EQ(transport->sentFrames.back().data[0], 0x80);

    CanTransport::Frame frame {};
    frame.id = 0x241;
    frame.dlc = 8;
    frame.isExtended = false;
    frame.isRemoteRequest = false;
    frame.data[0] = 0x9A;
    frame.data[6] = 0x01;
    frame.data[7] = 0x00;
    transport->simulateReceive(frame);
    EXPECT_TRUE(mt.hasFault(kMotorId));
}

TEST_F(MtCanTest, DISABLED_TODO_HandleResponseErrorCodeBranch)
{
    GTEST_SKIP() << "TODO: cover command 0x9A error decode and state transition.";
}

TEST_F(MtCanTest, DISABLED_TODO_HandleResponseResetBranch)
{
    GTEST_SKIP() << "TODO: cover command 0x64 reset flow and emitted frame.";
}
