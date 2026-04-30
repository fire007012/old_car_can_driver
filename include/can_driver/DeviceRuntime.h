#ifndef CAN_DRIVER_DEVICE_RUNTIME_H
#define CAN_DRIVER_DEVICE_RUNTIME_H

#include "can_driver/CanTxDispatcher.h"
#include "can_driver/SharedDriverState.h"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

class DeviceRuntime final : public CanTxDispatcher {
public:
    struct Options {
        std::size_t maxControlQueueDepth{256};
        std::size_t maxRecoverQueueDepth{128};
        std::size_t maxConfigQueueDepth{128};
        std::size_t maxQueryQueueDepth{64};
        /// Maximum consecutive backpressure events before the worker sleeps.
        std::size_t maxBackpressureRetries{3};
        /// How long to sleep after maxBackpressureRetries consecutive EAGAIN.
        std::chrono::microseconds backpressureSleepUs{500};

        bool autostart{true};
    };

    struct Stats {
        std::uint64_t submitted{0};
        std::uint64_t sent{0};
        std::uint64_t sendBackpressure{0};
        std::uint64_t sendLinkDown{0};
        std::uint64_t sendError{0};
        std::uint64_t droppedControl{0};
        std::uint64_t droppedRecover{0};
        std::uint64_t droppedConfig{0};
        std::uint64_t droppedQuery{0};
        std::uint64_t evictedControl{0};
        std::size_t pendingControl{0};
        std::size_t pendingRecover{0};
        std::size_t pendingConfig{0};
        std::size_t pendingQuery{0};
        std::size_t pendingTotal{0};
        bool workerRunning{false};
    };

    DeviceRuntime(std::shared_ptr<CanTransport> transport,
                  std::string deviceName);
    DeviceRuntime(std::shared_ptr<CanTransport> transport,
                  std::string deviceName,
                  Options options);
    ~DeviceRuntime() override;

    void submit(const Request &request) override;

    void start();
    void shutdown();
    bool waitUntilIdleFor(std::chrono::milliseconds timeout);
    Stats snapshotStats() const;
    void setSharedDriverState(std::shared_ptr<can_driver::SharedDriverState> sharedState);

private:
    using Queue = std::deque<Request>;

    void workerLoop();
    void noteSharedSendResult(CanTransport::SendResult result);
    bool allQueuesEmptyLocked() const;
    bool popNextLocked(Request *request);
    bool enqueueLocked(const Request &request);
    void completeDropped(const Request &request) const;
    Queue &queueFor(Category category);
    const Queue &queueFor(Category category) const;
    std::size_t maxDepthFor(Category category) const;
    const char *categoryName(Category category) const;
    void enqueueControlLocked(const Request &request);

    std::shared_ptr<CanTransport> transport_;
    std::shared_ptr<can_driver::SharedDriverState> sharedState_;
    const std::string deviceName_;
    const Options options_;

    mutable std::mutex mutex_;
    std::condition_variable cv_;
    std::condition_variable idleCv_;
    std::thread workerThread_;
    Queue controlQueue_;
    Queue recoverQueue_;
    Queue configQueue_;
    Queue queryQueue_;
    bool started_{false};
    bool stopRequested_{false};
    bool sending_{false};

    std::atomic<std::uint64_t> submittedCount_{0};
    std::atomic<std::uint64_t> sentCount_{0};
    std::atomic<std::uint64_t> sendBackpressureCount_{0};
    std::atomic<std::uint64_t> sendLinkDownCount_{0};
    std::atomic<std::uint64_t> sendErrorCount_{0};
    std::atomic<std::uint64_t> droppedControlCount_{0};
    std::atomic<std::uint64_t> droppedRecoverCount_{0};
    std::atomic<std::uint64_t> droppedConfigCount_{0};
    std::atomic<std::uint64_t> droppedQueryCount_{0};
    std::atomic<std::uint64_t> evictedControlCount_{0};
};

#endif // CAN_DRIVER_DEVICE_RUNTIME_H
