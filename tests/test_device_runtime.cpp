#include <gtest/gtest.h>
#include <ros/time.h>

#include "can_driver/DeviceRuntime.h"
#include "can_driver/SharedDriverState.h"

#include <chrono>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <set>
#include <thread>
#include <vector>

namespace {

class MockTransport : public CanTransport {
public:
    SendResult send(const Frame &frame) override
    {
        std::unique_lock<std::mutex> lock(mutex_);
        if (blockNextSend_) {
            blockNextSend_ = false;
            sendBlocked_ = true;
            cv_.notify_all();
            cv_.wait(lock, [this]() { return releaseBlockedSend_; });
            releaseBlockedSend_ = false;
        }
        auto result = nextResult_;
        if (!queuedResults_.empty()) {
            result = queuedResults_.front();
            queuedResults_.pop_front();
        }
        if (result == SendResult::Ok) {
            sentFrames_.push_back(frame);
            sentTimes_.push_back(std::chrono::steady_clock::now());
        }
        senderThreads_.insert(std::this_thread::get_id());
        ++sendCallCount_;
        return result;
    }

    std::size_t addReceiveHandler(ReceiveHandler handler) override
    {
        std::lock_guard<std::mutex> lock(mutex_);
        receiveHandler_ = std::move(handler);
        return 1;
    }

    void removeReceiveHandler(std::size_t) override
    {
        std::lock_guard<std::mutex> lock(mutex_);
        receiveHandler_ = nullptr;
    }

    std::vector<Frame> snapshotFrames() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return sentFrames_;
    }

    std::vector<std::chrono::steady_clock::time_point> snapshotSendTimes() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return sentTimes_;
    }

    std::size_t senderThreadCount() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return senderThreads_.size();
    }

    std::size_t totalSendCalls() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return sendCallCount_;
    }

    void blockNextSend()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        blockNextSend_ = true;
        sendBlocked_ = false;
        releaseBlockedSend_ = false;
    }

    bool waitUntilSendBlockedFor(std::chrono::milliseconds timeout)
    {
        std::unique_lock<std::mutex> lock(mutex_);
        return cv_.wait_for(lock, timeout, [this]() { return sendBlocked_; });
    }

    void releaseBlockedSend()
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            releaseBlockedSend_ = true;
        }
        cv_.notify_all();
    }

    void setNextResult(SendResult result)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        nextResult_ = result;
    }

    void queueResult(SendResult result)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        queuedResults_.push_back(result);
    }

private:
    mutable std::mutex mutex_;
    std::condition_variable cv_;
    std::vector<Frame> sentFrames_;
    std::vector<std::chrono::steady_clock::time_point> sentTimes_;
    std::set<std::thread::id> senderThreads_;
    std::size_t sendCallCount_{0};
    SendResult nextResult_{SendResult::Ok};
    std::deque<SendResult> queuedResults_;
    ReceiveHandler receiveHandler_;
    bool blockNextSend_{false};
    bool sendBlocked_{false};
    bool releaseBlockedSend_{false};
};

CanTransport::Frame makeFrame(std::uint32_t id)
{
    CanTransport::Frame frame;
    frame.id = id;
    frame.dlc = 1;
    frame.data[0] = static_cast<std::uint8_t>(id & 0xFF);
    return frame;
}

class DeviceRuntimeTest : public ::testing::Test {
protected:
    static void SetUpTestSuite()
    {
        ros::Time::init();
    }
};

} // namespace

TEST_F(DeviceRuntimeTest, DrainsQueuedFramesInPriorityOrder)
{
    auto transport = std::make_shared<MockTransport>();
    DeviceRuntime::Options options;
    options.autostart = false;
    DeviceRuntime runtime(transport, "test_can0", options);

    runtime.submit({makeFrame(0x04), CanTxDispatcher::Category::Query, "query"});
    runtime.submit({makeFrame(0x03), CanTxDispatcher::Category::Config, "config"});
    runtime.submit({makeFrame(0x02), CanTxDispatcher::Category::Recover, "recover"});
    runtime.submit({makeFrame(0x01), CanTxDispatcher::Category::Control, "control"});

    runtime.start();
    ASSERT_TRUE(runtime.waitUntilIdleFor(std::chrono::milliseconds(200)));

    const auto frames = transport->snapshotFrames();
    ASSERT_EQ(frames.size(), 4u);
    EXPECT_EQ(frames[0].id, 0x01u);
    EXPECT_EQ(frames[1].id, 0x02u);
    EXPECT_EQ(frames[2].id, 0x03u);
    EXPECT_EQ(frames[3].id, 0x04u);
}

TEST_F(DeviceRuntimeTest, DropsQueryFramesWhenQueryQueueIsFull)
{
    auto transport = std::make_shared<MockTransport>();
    DeviceRuntime::Options options;
    options.autostart = false;
    options.maxQueryQueueDepth = 1;
    DeviceRuntime runtime(transport, "test_can0", options);

    runtime.submit({makeFrame(0x11), CanTxDispatcher::Category::Query, "query1"});
    runtime.submit({makeFrame(0x12), CanTxDispatcher::Category::Query, "query2"});
    runtime.submit({makeFrame(0x10), CanTxDispatcher::Category::Control, "control"});

    runtime.start();
    ASSERT_TRUE(runtime.waitUntilIdleFor(std::chrono::milliseconds(200)));

    const auto frames = transport->snapshotFrames();
    ASSERT_EQ(frames.size(), 2u);
    EXPECT_EQ(frames[0].id, 0x10u);
    EXPECT_EQ(frames[1].id, 0x11u);

    const auto stats = runtime.snapshotStats();
    EXPECT_EQ(stats.droppedQuery, 1u);
}

TEST_F(DeviceRuntimeTest, UsesSingleWorkerThreadForAllTransmissions)
{
    auto transport = std::make_shared<MockTransport>();
    DeviceRuntime runtime(transport, "test_can0");

    std::vector<std::thread> producers;
    for (std::uint32_t producer = 0; producer < 4; ++producer) {
        producers.emplace_back([&runtime, producer]() {
            for (std::uint32_t i = 0; i < 8; ++i) {
                runtime.submit({makeFrame(0x100 + producer * 8 + i),
                                CanTxDispatcher::Category::Control,
                                "producer"});
            }
        });
    }

    for (auto &producer : producers) {
        producer.join();
    }

    ASSERT_TRUE(runtime.waitUntilIdleFor(std::chrono::milliseconds(500)));

    const auto frames = transport->snapshotFrames();
    EXPECT_EQ(frames.size(), 32u);
    EXPECT_EQ(transport->senderThreadCount(), 1u);
}

TEST_F(DeviceRuntimeTest, ControlQueueEvictsOldestWhenFull)
{
    auto transport = std::make_shared<MockTransport>();
    DeviceRuntime::Options options;
    options.autostart = false;
    options.maxControlQueueDepth = 2;
    DeviceRuntime runtime(transport, "test_can0", options);

    runtime.submit({makeFrame(0x01), CanTxDispatcher::Category::Control, "old"});
    runtime.submit({makeFrame(0x02), CanTxDispatcher::Category::Control, "mid"});
    runtime.submit({makeFrame(0x03), CanTxDispatcher::Category::Control, "new"});

    runtime.start();
    ASSERT_TRUE(runtime.waitUntilIdleFor(std::chrono::milliseconds(200)));

    const auto frames = transport->snapshotFrames();
    ASSERT_EQ(frames.size(), 2u);
    EXPECT_EQ(frames[0].id, 0x02u);
    EXPECT_EQ(frames[1].id, 0x03u);

    const auto stats = runtime.snapshotStats();
    EXPECT_EQ(stats.evictedControl, 1u);
    EXPECT_EQ(stats.droppedControl, 0u);
}

TEST_F(DeviceRuntimeTest, ControlQueueReplacesOlderFrameForSameCanId)
{
    auto transport = std::make_shared<MockTransport>();
    DeviceRuntime::Options options;
    options.autostart = false;
    DeviceRuntime runtime(transport, "test_can0", options);

    auto oldLeft = makeFrame(0x141);
    oldLeft.data[0] = 0x01;
    auto right = makeFrame(0x142);
    right.data[0] = 0x02;
    auto newLeft = makeFrame(0x141);
    newLeft.data[0] = 0x03;

    runtime.submit({oldLeft, CanTxDispatcher::Category::Control, "old_left"});
    runtime.submit({right, CanTxDispatcher::Category::Control, "right"});
    runtime.submit({newLeft, CanTxDispatcher::Category::Control, "new_left"});

    runtime.start();
    ASSERT_TRUE(runtime.waitUntilIdleFor(std::chrono::milliseconds(200)));

    const auto frames = transport->snapshotFrames();
    ASSERT_EQ(frames.size(), 2u);
    EXPECT_EQ(frames[0].id, 0x141u);
    EXPECT_EQ(frames[0].data[0], 0x03u);
    EXPECT_EQ(frames[1].id, 0x142u);

    const auto stats = runtime.snapshotStats();
    EXPECT_EQ(stats.evictedControl, 1u);
    EXPECT_EQ(stats.droppedControl, 0u);
}

TEST_F(DeviceRuntimeTest, DrainsReadyControlBurstBeforeLowerPriorityFrames)
{
    auto transport = std::make_shared<MockTransport>();
    DeviceRuntime::Options options;
    options.autostart = false;
    DeviceRuntime runtime(transport, "test_can0", options);

    runtime.submit({makeFrame(0x150), CanTxDispatcher::Category::Query, "state_query"});
    runtime.submit({makeFrame(0x141), CanTxDispatcher::Category::Control, "left_wheel"});
    runtime.submit({makeFrame(0x142), CanTxDispatcher::Category::Control, "right_wheel"});

    runtime.start();
    ASSERT_TRUE(runtime.waitUntilIdleFor(std::chrono::milliseconds(200)));

    const auto frames = transport->snapshotFrames();
    ASSERT_EQ(frames.size(), 3u);
    EXPECT_EQ(frames[0].id, 0x141u);
    EXPECT_EQ(frames[1].id, 0x142u);
    EXPECT_EQ(frames[2].id, 0x150u);
}

TEST_F(DeviceRuntimeTest, AppliesGapBetweenConsecutiveControlFrames)
{
    auto transport = std::make_shared<MockTransport>();
    DeviceRuntime::Options options;
    options.autostart = false;
    options.controlInterFrameGapUs = std::chrono::microseconds(3000);
    DeviceRuntime runtime(transport, "test_can0", options);

    runtime.submit({makeFrame(0x141), CanTxDispatcher::Category::Control, "left_wheel"});
    runtime.submit({makeFrame(0x142), CanTxDispatcher::Category::Control, "right_wheel"});

    runtime.start();
    ASSERT_TRUE(runtime.waitUntilIdleFor(std::chrono::milliseconds(300)));

    const auto frames = transport->snapshotFrames();
    ASSERT_EQ(frames.size(), 2u);
    EXPECT_EQ(frames[0].id, 0x141u);
    EXPECT_EQ(frames[1].id, 0x142u);

    const auto sendTimes = transport->snapshotSendTimes();
    ASSERT_EQ(sendTimes.size(), 2u);
    const auto gapUs = std::chrono::duration_cast<std::chrono::microseconds>(
        sendTimes[1] - sendTimes[0]);
    EXPECT_GE(gapUs.count(), 2500);
}

TEST_F(DeviceRuntimeTest, CoalescesTrailingControlFrameBeforeQueuedQuery)
{
    auto transport = std::make_shared<MockTransport>();
    DeviceRuntime::Options options;
    options.autostart = false;
    options.controlBurstCoalesceUs = std::chrono::microseconds(5000);
    DeviceRuntime runtime(transport, "test_can0", options);

    transport->blockNextSend();
    runtime.start();
    runtime.submit({makeFrame(0x141), CanTxDispatcher::Category::Control, "left_wheel"});
    ASSERT_TRUE(transport->waitUntilSendBlockedFor(std::chrono::milliseconds(200)));

    runtime.submit({makeFrame(0x150), CanTxDispatcher::Category::Query, "state_query"});
    transport->releaseBlockedSend();
    runtime.submit({makeFrame(0x142), CanTxDispatcher::Category::Control, "right_wheel"});

    ASSERT_TRUE(runtime.waitUntilIdleFor(std::chrono::milliseconds(200)));

    const auto frames = transport->snapshotFrames();
    ASSERT_EQ(frames.size(), 3u);
    EXPECT_EQ(frames[0].id, 0x141u);
    EXPECT_EQ(frames[1].id, 0x142u);
    EXPECT_EQ(frames[2].id, 0x150u);
}

TEST_F(DeviceRuntimeTest, BackpressureCountedInStats)
{
    auto transport = std::make_shared<MockTransport>();
    DeviceRuntime::Options options;
    options.autostart = false;
    options.maxBackpressureRetries = 100;
    DeviceRuntime runtime(transport, "test_can0", options);

    transport->setNextResult(CanTransport::SendResult::Backpressure);
    runtime.submit({makeFrame(0x01), CanTxDispatcher::Category::Control, "bp_frame"});

    runtime.start();
    ASSERT_TRUE(runtime.waitUntilIdleFor(std::chrono::milliseconds(200)));

    const auto stats = runtime.snapshotStats();
    EXPECT_EQ(stats.sendBackpressure, 1u);
    EXPECT_EQ(stats.sent, 0u);
    EXPECT_EQ(stats.submitted, 1u);
}

TEST_F(DeviceRuntimeTest, LinkDownCountedInStats)
{
    auto transport = std::make_shared<MockTransport>();
    DeviceRuntime::Options options;
    options.autostart = false;
    DeviceRuntime runtime(transport, "test_can0", options);

    transport->setNextResult(CanTransport::SendResult::LinkDown);
    runtime.submit({makeFrame(0x01), CanTxDispatcher::Category::Query, "link_test"});

    runtime.start();
    ASSERT_TRUE(runtime.waitUntilIdleFor(std::chrono::milliseconds(200)));

    const auto stats = runtime.snapshotStats();
    EXPECT_EQ(stats.sendLinkDown, 1u);
    EXPECT_EQ(stats.sent, 0u);
}

TEST_F(DeviceRuntimeTest, BackpressureUpdatesSharedDeviceHealth)
{
    auto transport = std::make_shared<MockTransport>();
    auto sharedState = std::make_shared<can_driver::SharedDriverState>();
    DeviceRuntime::Options options;
    options.autostart = false;
    DeviceRuntime runtime(transport, "test_can0", options);
    runtime.setSharedDriverState(sharedState);

    transport->setNextResult(CanTransport::SendResult::Backpressure);
    runtime.submit({makeFrame(0x01), CanTxDispatcher::Category::Control, "bp_shared"});

    runtime.start();
    ASSERT_TRUE(runtime.waitUntilIdleFor(std::chrono::milliseconds(200)));

    can_driver::SharedDriverState::DeviceHealthState health;
    ASSERT_TRUE(sharedState->getDeviceHealth("test_can0", &health));
    EXPECT_EQ(health.txBackpressure, 1u);
    EXPECT_EQ(health.txLinkUnavailable, 0u);
    EXPECT_EQ(health.txError, 0u);
}

TEST_F(DeviceRuntimeTest, LinkDownUpdatesSharedDeviceHealthImmediately)
{
    auto transport = std::make_shared<MockTransport>();
    auto sharedState = std::make_shared<can_driver::SharedDriverState>();
    DeviceRuntime::Options options;
    options.autostart = false;
    DeviceRuntime runtime(transport, "test_can0", options);
    runtime.setSharedDriverState(sharedState);

    transport->setNextResult(CanTransport::SendResult::LinkDown);
    runtime.submit({makeFrame(0x01), CanTxDispatcher::Category::Query, "link_shared"});

    runtime.start();
    ASSERT_TRUE(runtime.waitUntilIdleFor(std::chrono::milliseconds(200)));

    can_driver::SharedDriverState::DeviceHealthState health;
    ASSERT_TRUE(sharedState->getDeviceHealth("test_can0", &health));
    EXPECT_FALSE(health.transportReady);
    EXPECT_EQ(health.txLinkUnavailable, 1u);
    EXPECT_GT(health.lastTxLinkUnavailableSteadyNs, 0);
}

TEST_F(DeviceRuntimeTest, MixedResultsCountedCorrectly)
{
    auto transport = std::make_shared<MockTransport>();
    DeviceRuntime::Options options;
    options.autostart = false;
    options.maxBackpressureRetries = 100;
    DeviceRuntime runtime(transport, "test_can0", options);

    runtime.submit({makeFrame(0x01), CanTxDispatcher::Category::Control, "will_fail"});
    runtime.submit({makeFrame(0x02), CanTxDispatcher::Category::Query, "will_ok"});
    transport->queueResult(CanTransport::SendResult::Backpressure);
    transport->queueResult(CanTransport::SendResult::Ok);

    runtime.start();
    ASSERT_TRUE(runtime.waitUntilIdleFor(std::chrono::milliseconds(500)));

    const auto stats = runtime.snapshotStats();
    EXPECT_EQ(stats.submitted, 2u);
    EXPECT_EQ(stats.sendBackpressure, 1u);
    EXPECT_EQ(stats.sent, 1u);
    EXPECT_EQ(transport->totalSendCalls(), 2u);
}
