#include "can_driver/DeviceRuntime.h"

#include <ros/ros.h>

#include <utility>

DeviceRuntime::DeviceRuntime(std::shared_ptr<CanTransport> transport,
                             std::string deviceName)
    : DeviceRuntime(std::move(transport), std::move(deviceName), Options {})
{
}

DeviceRuntime::DeviceRuntime(std::shared_ptr<CanTransport> transport,
                             std::string deviceName,
                             Options options)
    : transport_(std::move(transport))
    , deviceName_(std::move(deviceName))
    , options_(options)
{
    if (options_.autostart) {
        start();
    }
}

DeviceRuntime::~DeviceRuntime()
{
    shutdown();
}

void DeviceRuntime::submit(const Request &request)
{
    bool accepted = false;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        accepted = enqueueLocked(request);
    }

    if (!accepted) {
        completeDropped(request);
        ROS_WARN_STREAM_THROTTLE(
            1.0,
            "[DeviceRuntime] Dropping " << categoryName(request.category)
            << " frame on " << deviceName_
            << " because queue is full or runtime is stopping"
            << " stats={submitted=" << submittedCount_.load(std::memory_order_relaxed)
            << ", sent=" << sentCount_.load(std::memory_order_relaxed)
            << ", dropped_query=" << droppedQueryCount_.load(std::memory_order_relaxed)
            << "}");
        return;
    }

    cv_.notify_one();
}

bool DeviceRuntime::enqueueLocked(const Request &request)
{
    if (stopRequested_) {
        switch (request.category) {
        case Category::Control:
            droppedControlCount_.fetch_add(1, std::memory_order_relaxed);
            break;
        case Category::Recover:
            droppedRecoverCount_.fetch_add(1, std::memory_order_relaxed);
            break;
        case Category::Config:
            droppedConfigCount_.fetch_add(1, std::memory_order_relaxed);
            break;
        case Category::Query:
            droppedQueryCount_.fetch_add(1, std::memory_order_relaxed);
            break;
        }
        return false;
    }

    if (request.category == Category::Control) {
        enqueueControlLocked(request);
        submittedCount_.fetch_add(1, std::memory_order_relaxed);
        return true;
    }

    Queue &queue = queueFor(request.category);
    if (queue.size() >= maxDepthFor(request.category)) {
        switch (request.category) {
        case Category::Control:
            break;
        case Category::Recover:
            droppedRecoverCount_.fetch_add(1, std::memory_order_relaxed);
            break;
        case Category::Config:
            droppedConfigCount_.fetch_add(1, std::memory_order_relaxed);
            break;
        case Category::Query:
            droppedQueryCount_.fetch_add(1, std::memory_order_relaxed);
            break;
        }
        return false;
    }

    queue.push_back(request);
    submittedCount_.fetch_add(1, std::memory_order_relaxed);
    return true;
}

void DeviceRuntime::completeDropped(const Request &request) const
{
    if (request.completion) {
        request.completion(false,
                           CanTransport::SendResult::Error,
                           std::chrono::steady_clock::now());
    }
}

void DeviceRuntime::enqueueControlLocked(const Request &request)
{
    for (auto &queued : controlQueue_) {
        if (queued.frame.id == request.frame.id) {
            queued = request;
            evictedControlCount_.fetch_add(1, std::memory_order_relaxed);
            return;
        }
    }

    if (controlQueue_.size() >= options_.maxControlQueueDepth) {
        // Evict the oldest control frame to make room for the newest.
        controlQueue_.pop_front();
        evictedControlCount_.fetch_add(1, std::memory_order_relaxed);
    }
    controlQueue_.push_back(request);
}

void DeviceRuntime::start()
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (started_) {
        return;
    }

    stopRequested_ = false;
    started_ = true;
    workerThread_ = std::thread(&DeviceRuntime::workerLoop, this);
}

void DeviceRuntime::setSharedDriverState(std::shared_ptr<can_driver::SharedDriverState> sharedState)
{
    std::lock_guard<std::mutex> lock(mutex_);
    sharedState_ = std::move(sharedState);
}

void DeviceRuntime::shutdown()
{
    {
        std::lock_guard<std::mutex> lock(mutex_);
        stopRequested_ = true;
    }
    cv_.notify_all();
    idleCv_.notify_all();
    if (workerThread_.joinable()) {
        workerThread_.join();
    }
    {
        std::lock_guard<std::mutex> lock(mutex_);
        started_ = false;
        sending_ = false;
    }
}

bool DeviceRuntime::waitUntilIdleFor(std::chrono::milliseconds timeout)
{
    std::unique_lock<std::mutex> lock(mutex_);
    return idleCv_.wait_for(lock, timeout, [this]() {
        return allQueuesEmptyLocked() && !sending_;
    });
}

DeviceRuntime::Stats DeviceRuntime::snapshotStats() const
{
    Stats stats;
    stats.submitted = submittedCount_.load(std::memory_order_relaxed);
    stats.sent = sentCount_.load(std::memory_order_relaxed);
    stats.sendBackpressure = sendBackpressureCount_.load(std::memory_order_relaxed);
    stats.sendLinkDown = sendLinkDownCount_.load(std::memory_order_relaxed);
    stats.sendError = sendErrorCount_.load(std::memory_order_relaxed);
    stats.droppedControl = droppedControlCount_.load(std::memory_order_relaxed);
    stats.droppedRecover = droppedRecoverCount_.load(std::memory_order_relaxed);
    stats.droppedConfig = droppedConfigCount_.load(std::memory_order_relaxed);
    stats.droppedQuery = droppedQueryCount_.load(std::memory_order_relaxed);
    stats.evictedControl = evictedControlCount_.load(std::memory_order_relaxed);
    {
        std::lock_guard<std::mutex> lock(mutex_);
        stats.pendingControl = controlQueue_.size();
        stats.pendingRecover = recoverQueue_.size();
        stats.pendingConfig = configQueue_.size();
        stats.pendingQuery = queryQueue_.size();
        stats.pendingTotal = stats.pendingControl + stats.pendingRecover +
                             stats.pendingConfig + stats.pendingQuery;
        stats.workerRunning = started_ && !stopRequested_;
    }
    return stats;
}

void DeviceRuntime::workerLoop()
{
    std::size_t consecutiveBackpressure = 0;

    for (;;) {
        Request request;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            cv_.wait(lock, [this]() {
                return stopRequested_ || !allQueuesEmptyLocked();
            });

            if (stopRequested_ && allQueuesEmptyLocked()) {
                sending_ = false;
                idleCv_.notify_all();
                break;
            }

            if (!popNextLocked(&request)) {
                continue;
            }
            sending_ = true;
        }

        // --- Attempt send and handle result ---
        CanTransport::SendResult result = CanTransport::SendResult::Error;
        const auto sendTime = std::chrono::steady_clock::now();
        if (transport_) {
            result = transport_->send(request.frame);
        }
        if (request.completion) {
            request.completion(true, result, sendTime);
        }

        switch (result) {
        case CanTransport::SendResult::Ok:
            sentCount_.fetch_add(1, std::memory_order_relaxed);
            consecutiveBackpressure = 0;
            break;

        case CanTransport::SendResult::Backpressure:
            sendBackpressureCount_.fetch_add(1, std::memory_order_relaxed);
            ++consecutiveBackpressure;
            if (consecutiveBackpressure >= options_.maxBackpressureRetries) {
                // Back off: sleep briefly to let the kernel TX queue drain.
                ROS_WARN_STREAM_THROTTLE(
                    1.0,
                    "[DeviceRuntime] " << consecutiveBackpressure
                    << " consecutive backpressure events on " << deviceName_
                    << ", sleeping " << options_.backpressureSleepUs.count() << "us");
                std::this_thread::sleep_for(options_.backpressureSleepUs);
                consecutiveBackpressure = 0;
            }
            // Frame is lost (already counted by transport); move on.
            break;

        case CanTransport::SendResult::LinkDown:
            sendLinkDownCount_.fetch_add(1, std::memory_order_relaxed);
            consecutiveBackpressure = 0;
            // Frame is lost; link-level recovery is outside our scope.
            break;

        case CanTransport::SendResult::Error:
            sendErrorCount_.fetch_add(1, std::memory_order_relaxed);
            consecutiveBackpressure = 0;
            break;
        }

        noteSharedSendResult(result);

        {
            std::lock_guard<std::mutex> lock(mutex_);
            sending_ = false;
        }
        idleCv_.notify_all();
    }
}

void DeviceRuntime::noteSharedSendResult(CanTransport::SendResult result)
{
    std::shared_ptr<can_driver::SharedDriverState> sharedState;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        sharedState = sharedState_;
    }
    if (!sharedState || deviceName_.empty()) {
        return;
    }

    switch (result) {
    case CanTransport::SendResult::Ok:
        return;
    case CanTransport::SendResult::Backpressure:
        sharedState->mutateDeviceHealth(
            deviceName_,
            [](can_driver::SharedDriverState::DeviceHealthState *health) {
                ++health->txBackpressure;
            });
        return;
    case CanTransport::SendResult::LinkDown:
        sharedState->mutateDeviceHealth(
            deviceName_,
            [](can_driver::SharedDriverState::DeviceHealthState *health) {
                ++health->txLinkUnavailable;
                health->transportReady = false;
                health->lastTxLinkUnavailableSteadyNs = can_driver::SharedDriverSteadyNowNs();
            });
        return;
    case CanTransport::SendResult::Error:
        sharedState->mutateDeviceHealth(
            deviceName_,
            [](can_driver::SharedDriverState::DeviceHealthState *health) {
                ++health->txError;
            });
        return;
    }
}

bool DeviceRuntime::allQueuesEmptyLocked() const
{
    return controlQueue_.empty() && recoverQueue_.empty() &&
           configQueue_.empty() && queryQueue_.empty();
}

bool DeviceRuntime::popNextLocked(Request *request)
{
    if (!controlQueue_.empty()) {
        *request = controlQueue_.front();
        controlQueue_.pop_front();
        return true;
    }
    if (!recoverQueue_.empty()) {
        *request = recoverQueue_.front();
        recoverQueue_.pop_front();
        return true;
    }
    if (!configQueue_.empty()) {
        *request = configQueue_.front();
        configQueue_.pop_front();
        return true;
    }
    if (!queryQueue_.empty()) {
        *request = queryQueue_.front();
        queryQueue_.pop_front();
        return true;
    }
    return false;
}

DeviceRuntime::Queue &DeviceRuntime::queueFor(Category category)
{
    switch (category) {
    case Category::Control:
        return controlQueue_;
    case Category::Recover:
        return recoverQueue_;
    case Category::Config:
        return configQueue_;
    case Category::Query:
        return queryQueue_;
    }
    return controlQueue_;
}

const DeviceRuntime::Queue &DeviceRuntime::queueFor(Category category) const
{
    switch (category) {
    case Category::Control:
        return controlQueue_;
    case Category::Recover:
        return recoverQueue_;
    case Category::Config:
        return configQueue_;
    case Category::Query:
        return queryQueue_;
    }
    return controlQueue_;
}

std::size_t DeviceRuntime::maxDepthFor(Category category) const
{
    switch (category) {
    case Category::Control:
        return options_.maxControlQueueDepth;
    case Category::Recover:
        return options_.maxRecoverQueueDepth;
    case Category::Config:
        return options_.maxConfigQueueDepth;
    case Category::Query:
        return options_.maxQueryQueueDepth;
    }
    return options_.maxControlQueueDepth;
}

const char *DeviceRuntime::categoryName(Category category) const
{
    switch (category) {
    case Category::Control:
        return "control";
    case Category::Recover:
        return "recover";
    case Category::Config:
        return "config";
    case Category::Query:
        return "query";
    }
    return "unknown";
}
