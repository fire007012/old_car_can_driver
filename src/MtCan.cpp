#include "can_driver/MtCan.h"
#include "can_driver/DeviceRuntime.h"

#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <sstream>

#include <ros/ros.h>

namespace {

using can_driver::motorIdFromMtProtocolNodeId;
using can_driver::toMtProtocolNodeId;

constexpr uint16_t kSendBaseId = 0x140;
constexpr uint16_t kResponseBaseId = 0x240;
constexpr int32_t kDefaultPositionSpeedDps = 100;
constexpr std::size_t kQueriesPerMotorPerCycle = 3;
constexpr int64_t kBaseReadTimeoutCycles = 3;
constexpr int64_t kMaxReadTimeoutCycles = 8;
constexpr auto kMinReadRequestTimeout = std::chrono::milliseconds(250);
constexpr auto kMaxReadRequestTimeout = std::chrono::milliseconds(1500);

int16_t readInt16LE(const CanTransport::Frame &frame, std::size_t index)
{
    if (index + 1 >= frame.dlc) {
        return 0;
    }
    const int value = static_cast<int>(frame.data[index]) |
                      (static_cast<int>(frame.data[index + 1]) << 8);
    return static_cast<int16_t>(value);
}

uint16_t readUInt16LE(const CanTransport::Frame &frame, std::size_t index)
{
    if (index + 1 >= frame.dlc) {
        return 0;
    }
    return static_cast<uint16_t>(static_cast<uint16_t>(frame.data[index]) |
                                 (static_cast<uint16_t>(frame.data[index + 1]) << 8));
}

int32_t readInt32LE(const CanTransport::Frame &frame, std::size_t index)
{
    if (index + 3 >= frame.dlc) {
        return 0;
    }
    const uint32_t value = static_cast<uint32_t>(frame.data[index]) |
                           (static_cast<uint32_t>(frame.data[index + 1]) << 8) |
                           (static_cast<uint32_t>(frame.data[index + 2]) << 16) |
                           (static_cast<uint32_t>(frame.data[index + 3]) << 24);
    return static_cast<int32_t>(value);
}

std::string formatCommandHex(uint8_t command)
{
    std::ostringstream stream;
    stream << std::hex << static_cast<unsigned>(command);
    return stream.str();
}

} // namespace

std::chrono::milliseconds MtCan::computeRefreshSleep(std::size_t motorCount) const
{
    const double hz = refreshRateHz_.load(std::memory_order_relaxed);
    if (std::isfinite(hz) && hz > 0.0) {
        const auto intervalMs = static_cast<int64_t>(std::llround(1000.0 / hz));
        return std::chrono::milliseconds(std::max<int64_t>(1, intervalMs));
    }
    const std::size_t intervalMs = std::max<std::size_t>(5, motorCount * kQueriesPerMotorPerCycle);
    return std::chrono::milliseconds(intervalMs);
}

MtCan::MtCan(std::shared_ptr<CanTransport> controller,
             std::shared_ptr<CanTxDispatcher> txDispatcher)
    : MtCan(std::move(controller), std::move(txDispatcher), nullptr, "")
{
}

MtCan::MtCan(std::shared_ptr<CanTransport> controller,
             std::shared_ptr<CanTxDispatcher> txDispatcher,
             std::shared_ptr<can_driver::SharedDriverState> sharedState,
             std::string deviceName)
    : canController(std::move(controller))
    , txDispatcher_(std::move(txDispatcher))
    , sharedState_(std::move(sharedState))
    , deviceName_(std::move(deviceName))
{
    if (canController) {
        receiveHandlerId = canController->addReceiveHandler(
            [this](const CanTransport::Frame &frame) { handleResponse(frame); });
    }
}

MtCan::~MtCan()
{
    shuttingDown_.store(true, std::memory_order_release);
    stopRefreshLoop();
    if (canController && receiveHandlerId != 0) {
        canController->removeReceiveHandler(receiveHandlerId);
        receiveHandlerId = 0;
    }
    if (const auto runtime = std::dynamic_pointer_cast<DeviceRuntime>(txDispatcher_)) {
        runtime->shutdown();
    }
    txDispatcher_.reset();
}

void MtCan::initializeMotorRefresh(const std::vector<MotorID> &motorIds)
{
    {
        std::lock_guard<std::mutex> lock(refreshMutex);
        refreshMotorIds.clear();
        systemMotorIdsByNodeId_.clear();
        refreshMotorIds.reserve(motorIds.size());
        for (MotorID id : motorIds) {
            const auto motorId = toMtProtocolNodeId(id);
            refreshMotorIds.push_back(motorId);
            systemMotorIdsByNodeId_[motorId] = id;
            if (sharedState_ && !deviceName_.empty()) {
                sharedState_->registerAxis(deviceName_, CanType::MT, id);
            }
        }
    }
    resetReadTracking();

    if (motorIds.empty()) {
        stopRefreshLoop();
        return;
    }

    // 电机注册后自动下发通讯中断保护，避免控制链路异常时失控
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    broadcastCommunicationTimeout(communicationTimeoutOnInitMs_.load(std::memory_order_relaxed));
}

void MtCan::setRefreshRateHz(double hz)
{
    if (!std::isfinite(hz) || hz <= 0.0) {
        refreshRateHz_.store(0.0, std::memory_order_relaxed);
        return;
    }
    refreshRateHz_.store(hz, std::memory_order_relaxed);
}

void MtCan::setCommunicationTimeoutOnInit(uint32_t timeoutMs)
{
    communicationTimeoutOnInitMs_.store(timeoutMs, std::memory_order_relaxed);
}

std::chrono::milliseconds MtCan::refreshSleepInterval() const
{
    std::size_t motorCount = 0;
    {
        std::lock_guard<std::mutex> lock(refreshMutex);
        motorCount = refreshMotorIds.size();
    }
    return computeRefreshSleep(std::max<std::size_t>(1, motorCount));
}

std::chrono::milliseconds MtCan::readResponseTimeout() const
{
    return computeReadRequestTimeout();
}

bool MtCan::setMode(MotorID Id, MotorMode mode)
{
    const uint8_t motorId = toMtProtocolNodeId(Id);
    rememberSystemMotorId(Id);
    {
        std::lock_guard<std::mutex> stateLock(stateMutex);
        motorStates[motorId].mode = mode;
    }
    syncSharedModeSelection(motorId, mode);
    return true;
}

bool MtCan::issueRefreshQuery(MotorID motorId, RefreshQuery query)
{
    rememberSystemMotorId(motorId);
    const uint8_t id = toMtProtocolNodeId(motorId);
    switch (query) {
    case RefreshQuery::State:
        return requestState(id);
    case RefreshQuery::MultiTurnAngle:
        return requestMultiTurnAngle(id);
    case RefreshQuery::Error:
        return requestError(id);
    }
    return false;
}

bool MtCan::setVelocity(MotorID Id, int32_t velocity)
{
    if (!canController) {
        return false;
    }
    const uint8_t motorId = toMtProtocolNodeId(Id);
    rememberSystemMotorId(Id);
    {
        std::lock_guard<std::mutex> stateLock(stateMutex);
        motorStates[motorId].commandedVelocity = velocity;
    }
    syncSharedCommand(motorId, 0, velocity, MotorMode::Velocity, true);
    const uint16_t canId = encodeSendCanId(motorId);

    std::array<uint8_t, 4> payload{
        static_cast<uint8_t>(velocity & 0xFF),
        static_cast<uint8_t>((velocity >> 8) & 0xFF),
        static_cast<uint8_t>((velocity >> 16) & 0xFF),
        static_cast<uint8_t>((velocity >> 24) & 0xFF)};
    sendFrame(canId, 0xA2, payload);
    return true;
}

bool MtCan::setAcceleration(MotorID Id, int32_t acceleration)
{
    if (acceleration <= 0) {
        acceleration = 100;
    }
    return setSpeedAcceleration(Id, static_cast<uint32_t>(acceleration));
}

bool MtCan::setDeceleration(MotorID Id, int32_t deceleration)
{
    if (deceleration <= 0) {
        deceleration = 100;
    }
    return setSpeedDeceleration(Id, static_cast<uint32_t>(deceleration));
}

bool MtCan::setCommunicationTimeout(uint32_t timeoutMs)
{
    if (!canController) {
        return false;
    }

    std::vector<uint8_t> motorIds;
    {
        std::lock_guard<std::mutex> lock(refreshMutex);
        motorIds = refreshMotorIds;
    }
    if (motorIds.empty()) {
        std::cerr << "[MtCan] setCommunicationTimeout: no motors registered\n";
        return false;
    }

    for (uint8_t motorId : motorIds) {
        const uint16_t canId = encodeSendCanId(motorId);
        CanTransport::Frame frame;
        frame.id = canId;
        frame.dlc = 8;
        frame.isExtended = false;
        frame.isRemoteRequest = false;
        frame.data.fill(0);
        frame.data[0] = 0xB3;
        frame.data[4] = static_cast<uint8_t>(timeoutMs & 0xFF);
        frame.data[5] = static_cast<uint8_t>((timeoutMs >> 8) & 0xFF);
        frame.data[6] = static_cast<uint8_t>((timeoutMs >> 16) & 0xFF);
        frame.data[7] = static_cast<uint8_t>((timeoutMs >> 24) & 0xFF);
        if (!submitTx(frame, CanTxDispatcher::Category::Config, "MtCan::setCommunicationTimeout")) {
            return false;
        }
    }

    std::cout << "[MtCan] Communication timeout set to " << timeoutMs
              << " ms for " << motorIds.size() << " motor(s)\n";
    return true;
}

bool MtCan::writeAcceleration(uint8_t motorId, uint8_t index, uint32_t value)
{
    if (!canController) {
        return false;
    }
    value = std::max<uint32_t>(100, std::min<uint32_t>(60000, value));

    const uint16_t canId = encodeSendCanId(motorId);
    CanTransport::Frame frame;
    frame.id = canId;
    frame.dlc = 8;
    frame.isExtended = false;
    frame.isRemoteRequest = false;
    frame.data.fill(0);
    frame.data[0] = 0x43;
    frame.data[1] = index;
    frame.data[4] = static_cast<uint8_t>(value & 0xFF);
    frame.data[5] = static_cast<uint8_t>((value >> 8) & 0xFF);
    frame.data[6] = static_cast<uint8_t>((value >> 16) & 0xFF);
    frame.data[7] = static_cast<uint8_t>((value >> 24) & 0xFF);
    return submitTx(frame, CanTxDispatcher::Category::Config, "MtCan::writeAcceleration");
}

bool MtCan::setSpeedAcceleration(MotorID id, uint32_t accelDpsPerSec)
{
    rememberSystemMotorId(id);
    return writeAcceleration(toMtProtocolNodeId(id), 0x02, accelDpsPerSec);
}

bool MtCan::setSpeedDeceleration(MotorID id, uint32_t decelDpsPerSec)
{
    rememberSystemMotorId(id);
    return writeAcceleration(toMtProtocolNodeId(id), 0x03, decelDpsPerSec);
}

bool MtCan::setPositionAcceleration(MotorID id, uint32_t accelDpsPerSec)
{
    rememberSystemMotorId(id);
    return writeAcceleration(toMtProtocolNodeId(id), 0x00, accelDpsPerSec);
}

bool MtCan::setPositionDeceleration(MotorID id, uint32_t decelDpsPerSec)
{
    rememberSystemMotorId(id);
    return writeAcceleration(toMtProtocolNodeId(id), 0x01, decelDpsPerSec);
}

void MtCan::broadcastCommunicationTimeout(uint32_t timeoutMs)
{
    (void)setCommunicationTimeout(timeoutMs);
}

bool MtCan::setPosition(MotorID Id, int32_t position)
{
    if (!canController) {
        return false;
    }
    const uint8_t motorId = toMtProtocolNodeId(Id);
    rememberSystemMotorId(Id);
    int32_t commandedVelocity = 0;
    {
        std::lock_guard<std::mutex> stateLock(stateMutex);
        MotorState &state = motorStates[motorId];
        state.position = position;
        commandedVelocity = state.commandedVelocity;
    }
    syncSharedCommand(motorId, position, commandedVelocity, MotorMode::Position, true);

    const uint16_t canId = encodeSendCanId(motorId);

    // 0xA4 DATA[2-3] = maxSpeed (uint16_t, 1 dps/LSB)
    // commandedVelocity 来自 0xA2 语义（0.01 dps/LSB），发送前需换算成 dps。
    std::int64_t absVel001Dps = std::llabs(static_cast<long long>(commandedVelocity));
    std::int64_t absVelDps = 0;
    if (absVel001Dps == 0) {
        absVelDps = kDefaultPositionSpeedDps;
    } else {
        absVelDps = (absVel001Dps + 50) / 100; // 四舍五入到 1 dps/LSB
        if (absVelDps == 0) {
            absVelDps = 1;
        }
    }
    const uint16_t maxSpeed = static_cast<uint16_t>(std::min<std::int64_t>(absVelDps, 0xFFFF));

    CanTransport::Frame frame;
    frame.id = canId;
    frame.dlc = 8;
    frame.isExtended = false;
    frame.isRemoteRequest = false;
    frame.data[0] = 0xA4;
    frame.data[1] = 0x00;
    frame.data[2] = static_cast<uint8_t>(maxSpeed & 0xFF);
    frame.data[3] = static_cast<uint8_t>((maxSpeed >> 8) & 0xFF);
    frame.data[4] = static_cast<uint8_t>(position & 0xFF);
    frame.data[5] = static_cast<uint8_t>((position >> 8) & 0xFF);
    frame.data[6] = static_cast<uint8_t>((position >> 16) & 0xFF);
    frame.data[7] = static_cast<uint8_t>((position >> 24) & 0xFF);

    return submitTx(frame, CanTxDispatcher::Category::Control, "MtCan::setPosition");
}

bool MtCan::quickSetPosition(MotorID Id, int32_t position)
{
    // MtCan 协议暂不支持 CSP 模式，此接口仅为满足基类要求
    // 如需使用 CSP 模式，请使用 EyouCan 协议
    (void)Id;
    (void)position;
    return false;
}

// [FIX #4] 不再每次 Enable 都设置零点并复位系统
bool MtCan::Enable(MotorID Id)
{
    if (!canController) {
        return false;
    }
    const uint8_t motorId = toMtProtocolNodeId(Id);
    rememberSystemMotorId(Id);
    MotorState stateSnapshot;
    {
        std::lock_guard<std::mutex> stateLock(stateMutex);
        MotorState &state = motorStates[motorId];
        state.enabled = true;
        state.enabledReceived = true;
        stateSnapshot = state;
    }
    syncSharedIntent(motorId, can_driver::AxisIntent::Enable);
    syncSharedFeedback(motorId, stateSnapshot);
    // 脉塔协议无独立使能命令。
    // 如需设置零点请单独调用 setZeroPosition()，避免频繁写 ROM。
    return true;
}

// [FIX #3] 使用 0x80 (Motor Off) 而非 0x81 (Stop)
bool MtCan::Disable(MotorID Id)
{
    if (!canController) {
        return false;
    }
    const uint8_t motorId = toMtProtocolNodeId(Id);
    rememberSystemMotorId(Id);
    MotorState stateSnapshot;
    {
        std::lock_guard<std::mutex> stateLock(stateMutex);
        MotorState &state = motorStates[motorId];
        state.enabled = false;
        state.enabledReceived = true;
        stateSnapshot = state;
    }
    syncSharedIntent(motorId, can_driver::AxisIntent::Disable);
    syncSharedFeedback(motorId, stateSnapshot);
    const uint16_t canId = encodeSendCanId(motorId);
    sendFrame(canId, 0x80, {0, 0, 0, 0}); // Motor Off: 关闭输出，清除运行状态
    return true;
}

bool MtCan::Stop(MotorID Id)
{
    if (!canController) {
        return false;
    }
    const uint8_t motorId = toMtProtocolNodeId(Id);
    rememberSystemMotorId(Id);
    syncSharedIntent(motorId, can_driver::AxisIntent::Hold);
    const uint16_t canId = encodeSendCanId(motorId);
    sendFrame(canId, 0x81, {0, 0, 0, 0}); // Motor Stop: 停止运动，保持受控
    return true;
}

bool MtCan::ResetFault(MotorID Id)
{
    if (!canController) {
        return false;
    }
    const uint8_t motorId = toMtProtocolNodeId(Id);
    rememberSystemMotorId(Id);
    syncSharedIntent(motorId, can_driver::AxisIntent::Recover);
    resetSystem(motorId);
    markReadResponseReceived(motorId, 0x9A);
    requestError(motorId);
    return true;
}

// [FIX #5] 返回电机实际位置（从 0x92 多圈角度读回），而非命令值
int64_t MtCan::getPosition(MotorID Id) const
{
    const uint8_t motorId = toMtProtocolNodeId(Id);
    {
        std::lock_guard<std::mutex> stateLock(stateMutex);
        auto it = motorStates.find(motorId);
        if (it != motorStates.end()) {
            return it->second.multiTurnAngle;
        }
    }
    return 0;
}

int16_t MtCan::getCurrent(MotorID Id) const
{
    const uint8_t motorId = toMtProtocolNodeId(Id);
    {
        std::lock_guard<std::mutex> stateLock(stateMutex);
        auto it = motorStates.find(motorId);
        if (it != motorStates.end()) {
            return static_cast<int16_t>(std::lround(it->second.current * 100));
        }
    }
    return 0;
}

// [FIX #7] 移除 velocity == 0 的不可靠刷新判断
int32_t MtCan::getVelocity(MotorID Id) const
{
    const uint8_t motorId = toMtProtocolNodeId(Id);
    {
        std::lock_guard<std::mutex> stateLock(stateMutex);
        auto it = motorStates.find(motorId);
        if (it != motorStates.end()) {
            return it->second.velocity;
        }
    }
    return 0;
}

bool MtCan::isEnabled(MotorID Id) const
{
    const uint8_t motorId = toMtProtocolNodeId(Id);
    std::lock_guard<std::mutex> stateLock(stateMutex);
    auto it = motorStates.find(motorId);
    return (it != motorStates.end()) ? it->second.enabled : false;
}

bool MtCan::hasFault(MotorID Id) const
{
    const uint8_t motorId = can_driver::toProtocolNodeId(Id);
    std::lock_guard<std::mutex> stateLock(stateMutex);
    auto it = motorStates.find(motorId);
    return (it != motorStates.end()) ? it->second.error : false;
}

uint16_t MtCan::encodeSendCanId(uint8_t motorId) const
{
    return static_cast<uint16_t>(kSendBaseId + motorId);
}

void MtCan::sendFrame(uint16_t canId, uint8_t command, const std::array<uint8_t, 4> &payload) const
{
    if (!canController) {
        std::cerr << "[MtCan] CAN controller not initialized\n";
        return;
    }

    CanTransport::Frame frame = makeCommandFrame(canId, command, payload);

    auto category = CanTxDispatcher::Category::Control;
    if (command == 0x76) {
        category = CanTxDispatcher::Category::Recover;
    } else if (command == 0x64 || command == 0xB3 || command == 0x43) {
        category = CanTxDispatcher::Category::Config;
    }
    (void)submitTx(frame, category, "MtCan::sendFrame");
}

CanTransport::Frame MtCan::makeCommandFrame(uint16_t canId,
                                            uint8_t command,
                                            const std::array<uint8_t, 4> &payload)
{
    CanTransport::Frame frame;
    frame.id = canId;
    frame.dlc = 8;
    frame.isExtended = false;
    frame.isRemoteRequest = false;
    frame.data[0] = command;
    frame.data[1] = 0x00;
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = payload[0];
    frame.data[5] = payload[1];
    frame.data[6] = payload[2];
    frame.data[7] = payload[3];
    return frame;
}

// [FIX #2] DLC 改为 8，数据全部清零
bool MtCan::requestState(uint8_t motorId)
{
    if (!canController) {
        return false;
    }
    if (!tryIssueReadCommand(motorId, 0x9C)) {
        return false;
    }
    const uint16_t canId = encodeSendCanId(motorId);

    CanTransport::Frame frame;
    frame.id = canId;
    frame.dlc = 8;
    frame.isExtended = false;
    frame.isRemoteRequest = false;
    frame.data.fill(0);
    frame.data[0] = 0x9C;

    if (!txDispatcher_) {
        ROS_ERROR_STREAM_THROTTLE(1.0,
                                  "[MtCan] TX dispatcher unavailable for MtCan::requestState");
        onReadDispatchResult(motorId,
                             0x9C,
                             false,
                             CanTransport::SendResult::Error,
                             std::chrono::steady_clock::now());
        return false;
    }

    CanTxDispatcher::Request request;
    request.frame = frame;
    request.category = CanTxDispatcher::Category::Query;
    request.source = "MtCan::requestState";
    request.completion = [this, motorId](bool attemptedSend,
                                         CanTransport::SendResult sendResult,
                                         std::chrono::steady_clock::time_point eventTime) {
        onReadDispatchResult(motorId, 0x9C, attemptedSend, sendResult, eventTime);
    };
    txDispatcher_->submit(request);
    return true;
}

// [FIX #2] DLC 改为 8，数据全部清零
bool MtCan::requestError(uint8_t motorId)
{
    if (!canController) {
        return false;
    }
    if (!tryIssueReadCommand(motorId, 0x9A)) {
        return false;
    }
    const uint16_t canId = encodeSendCanId(motorId);

    CanTransport::Frame frame;
    frame.id = canId;
    frame.dlc = 8;
    frame.isExtended = false;
    frame.isRemoteRequest = false;
    frame.data.fill(0);
    frame.data[0] = 0x9A;

    if (!txDispatcher_) {
        ROS_ERROR_STREAM_THROTTLE(1.0,
                                  "[MtCan] TX dispatcher unavailable for MtCan::requestError");
        onReadDispatchResult(motorId,
                             0x9A,
                             false,
                             CanTransport::SendResult::Error,
                             std::chrono::steady_clock::now());
        return false;
    }

    CanTxDispatcher::Request request;
    request.frame = frame;
    request.category = CanTxDispatcher::Category::Query;
    request.source = "MtCan::requestError";
    request.completion = [this, motorId](bool attemptedSend,
                                         CanTransport::SendResult sendResult,
                                         std::chrono::steady_clock::time_point eventTime) {
        onReadDispatchResult(motorId, 0x9A, attemptedSend, sendResult, eventTime);
    };
    txDispatcher_->submit(request);
    return true;
}

// [FIX #5 NEW] 请求多圈角度 (0x92) 以获取实际位置
bool MtCan::requestMultiTurnAngle(uint8_t motorId)
{
    if (!canController) {
        return false;
    }
    if (!tryIssueReadCommand(motorId, 0x92)) {
        return false;
    }
    const uint16_t canId = encodeSendCanId(motorId);

    CanTransport::Frame frame;
    frame.id = canId;
    frame.dlc = 8;
    frame.isExtended = false;
    frame.isRemoteRequest = false;
    frame.data.fill(0);
    frame.data[0] = 0x92;

    if (!txDispatcher_) {
        ROS_ERROR_STREAM_THROTTLE(
            1.0, "[MtCan] TX dispatcher unavailable for MtCan::requestMultiTurnAngle");
        onReadDispatchResult(motorId,
                             0x92,
                             false,
                             CanTransport::SendResult::Error,
                             std::chrono::steady_clock::now());
        return false;
    }

    CanTxDispatcher::Request request;
    request.frame = frame;
    request.category = CanTxDispatcher::Category::Query;
    request.source = "MtCan::requestMultiTurnAngle";
    request.completion = [this, motorId](bool attemptedSend,
                                         CanTransport::SendResult sendResult,
                                         std::chrono::steady_clock::time_point eventTime) {
        onReadDispatchResult(motorId, 0x92, attemptedSend, sendResult, eventTime);
    };
    txDispatcher_->submit(request);
    return true;
}

bool MtCan::submitTx(const CanTransport::Frame &frame,
                     CanTxDispatcher::Category category,
                     const char *source) const
{
    if (!txDispatcher_) {
        ROS_ERROR_STREAM_THROTTLE(1.0,
                                  "[MtCan] TX dispatcher unavailable for "
                                  << (source ? source : "unknown"));
        return false;
    }

    CanTxDispatcher::Request request;
    request.frame = frame;
    request.category = category;
    request.source = source;
    txDispatcher_->submit(request);
    return true;
}

void MtCan::resetSystem(uint8_t motorId) const
{
    const uint16_t canId = encodeSendCanId(motorId);
    sendFrame(canId, 0x76, {0, 0, 0, 0});
}

void MtCan::setZeroPosition(uint8_t motorId) const
{
    const uint16_t canId = encodeSendCanId(motorId);
    sendFrame(canId, 0x64, {0, 0, 0, 0});
}

void MtCan::stopRefreshLoop()
{
    resetReadTracking();
}

bool MtCan::tryIssueReadCommand(uint8_t motorId, uint8_t command)
{
    if (shuttingDown_.load(std::memory_order_acquire) || !canController) {
        return false;
    }

    const auto now = std::chrono::steady_clock::now();
    const auto timeout = computeReadRequestTimeout();
    bool delayRetry = false;
    std::size_t consecutiveTimeouts = 0;
    std::chrono::milliseconds retryBackoff(0);
    {
        std::lock_guard<std::mutex> lock(pendingReadMutex_);
        auto &request = pendingReadRequests_[pendingReadKey(motorId, command)];
        if (request.nextEligibleSend != std::chrono::steady_clock::time_point {} &&
            now < request.nextEligibleSend) {
            return false;
        }
        if (request.queued) {
            return false;
        }
        if (request.inFlight && (now - request.lastSent) < timeout) {
            return false;
        }
        if (request.inFlight) {
            request.inFlight = false;
            request.consecutiveTimeouts = std::min<std::size_t>(request.consecutiveTimeouts + 1, 8);
            request.nextEligibleSend = now + computeTimeoutBackoff(request.consecutiveTimeouts, timeout);
            consecutiveTimeouts = request.consecutiveTimeouts;
            retryBackoff = std::chrono::duration_cast<std::chrono::milliseconds>(
                request.nextEligibleSend - now);
            delayRetry = true;
        } else {
            request.queued = true;
            request.nextEligibleSend = std::chrono::steady_clock::time_point {};
        }
    }

    if (delayRetry) {
        noteSharedTimeout(motorId, consecutiveTimeouts);
        const auto message = "[MtCan] Read timeout on motor " +
                             std::to_string(static_cast<unsigned>(motorId)) +
                             " cmd=0x" + formatCommandHex(command) +
                             ", backing off for " + std::to_string(retryBackoff.count()) +
                             " ms before retry" +
                             " (consecutive_timeouts=" + std::to_string(consecutiveTimeouts) + ")";
        if (consecutiveTimeouts >= can_driver::kDefaultFeedbackDegradedTimeoutThreshold) {
            ROS_WARN_STREAM_THROTTLE(1.0, message);
        } else {
            ROS_DEBUG_STREAM_THROTTLE(1.0, message);
        }
        return false;
    }

    return true;
}

void MtCan::onReadDispatchResult(uint8_t motorId,
                                 uint8_t command,
                                 bool attemptedSend,
                                 CanTransport::SendResult sendResult,
                                 std::chrono::steady_clock::time_point eventTime)
{
    if (shuttingDown_.load(std::memory_order_acquire)) {
        return;
    }
    std::lock_guard<std::mutex> lock(pendingReadMutex_);
    auto it = pendingReadRequests_.find(pendingReadKey(motorId, command));
    if (it == pendingReadRequests_.end()) {
        return;
    }

    auto &request = it->second;
    request.queued = false;
    request.inFlight = false;
    if (attemptedSend && sendResult == CanTransport::SendResult::Ok) {
        request.inFlight = true;
        request.lastSent = eventTime;
    }
}

void MtCan::markReadResponseReceived(uint8_t motorId, uint8_t command)
{
    std::size_t recoveredTimeouts = 0;
    long long responseAgeMs = -1;
    {
        std::lock_guard<std::mutex> lock(pendingReadMutex_);
        auto it = pendingReadRequests_.find(pendingReadKey(motorId, command));
        if (it != pendingReadRequests_.end()) {
            recoveredTimeouts = it->second.consecutiveTimeouts;
            if (it->second.lastSent != std::chrono::steady_clock::time_point {}) {
                responseAgeMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                                    std::chrono::steady_clock::now() - it->second.lastSent)
                                    .count();
            }
            it->second.queued = false;
            it->second.inFlight = false;
            it->second.nextEligibleSend = std::chrono::steady_clock::time_point {};
            it->second.consecutiveTimeouts = 0;
        }
    }

    if (recoveredTimeouts > 0) {
        const auto message = "[MtCan] Read response recovered on motor " +
                             std::to_string(static_cast<unsigned>(motorId)) +
                             " cmd=0x" + formatCommandHex(command) +
                             " after " + std::to_string(recoveredTimeouts) +
                             " consecutive timeout(s)" +
                             " (response_age_ms=" + std::to_string(responseAgeMs) + ")";
        if (recoveredTimeouts >= can_driver::kDefaultFeedbackDegradedTimeoutThreshold) {
            ROS_WARN_STREAM_THROTTLE(1.0, message);
        } else {
            ROS_DEBUG_STREAM_THROTTLE(1.0, message);
        }
    }
}

can_driver::SharedDriverState::AxisKey MtCan::makeAxisKey(uint8_t motorId) const
{
    return can_driver::MakeAxisKey(deviceName_, CanType::MT, resolveSystemMotorId(motorId));
}

void MtCan::syncSharedFeedback(uint8_t motorId, const MotorState &state) const
{
    if (!sharedState_ || deviceName_.empty()) {
        return;
    }

    const auto nowNs = can_driver::SharedDriverSteadyNowNs();
    sharedState_->mutateAxisFeedback(
        makeAxisKey(motorId),
        [&](can_driver::SharedDriverState::AxisFeedbackState *feedback) {
            feedback->position = state.multiTurnAngle;
            feedback->velocity = state.velocity;
            feedback->current = static_cast<std::int32_t>(std::lround(state.current * 100.0));
            feedback->mode = state.mode;
            feedback->positionValid = state.positionReceived;
            feedback->velocityValid = state.velocityReceived;
            feedback->currentValid = state.currentReceived;
            feedback->modeValid = state.modeReceived;
            feedback->enabled = state.enabled;
            feedback->fault = state.error;
            feedback->enabledValid = state.enabledReceived;
            feedback->faultValid = state.faultReceived;
            feedback->feedbackSeen = true;
            feedback->lastRxSteadyNs = nowNs;
            feedback->lastValidStateSteadyNs = nowNs;
            feedback->consecutiveTimeoutCount = 0;
            feedback->degraded = false;
        });
}

void MtCan::syncSharedCommand(uint8_t motorId,
                              int64_t targetPosition,
                              int32_t targetVelocity,
                              MotorMode desiredMode,
                              bool valid) const
{
    if (!sharedState_ || deviceName_.empty()) {
        return;
    }

    const auto nowNs = can_driver::SharedDriverSteadyNowNs();
    sharedState_->mutateAxisCommand(
        makeAxisKey(motorId),
        [&](can_driver::SharedDriverState::AxisCommandState *command) {
            command->targetPosition = targetPosition;
            command->targetVelocity = targetVelocity;
            command->desiredMode = desiredMode;
            command->desiredModeValid = true;
            command->valid = valid;
            command->lastCommandSteadyNs = valid ? nowNs : 0;
        });
}

void MtCan::syncSharedModeSelection(uint8_t motorId, MotorMode desiredMode) const
{
    if (!sharedState_ || deviceName_.empty()) {
        return;
    }

    sharedState_->mutateAxisCommand(
        makeAxisKey(motorId),
        [desiredMode](can_driver::SharedDriverState::AxisCommandState *command) {
            command->targetPosition = 0;
            command->targetVelocity = 0;
            command->targetCurrent = 0;
            command->desiredMode = desiredMode;
            command->desiredModeValid = true;
            command->valid = false;
            command->lastCommandSteadyNs = 0;
        });
}

void MtCan::syncSharedIntent(uint8_t motorId, can_driver::AxisIntent intent) const
{
    if (!sharedState_ || deviceName_.empty()) {
        return;
    }
    sharedState_->setAxisIntent(makeAxisKey(motorId), intent);
}

void MtCan::noteSharedTimeout(uint8_t motorId, std::size_t consecutiveTimeouts) const
{
    if (!sharedState_ || deviceName_.empty()) {
        return;
    }

    sharedState_->mutateAxisFeedback(
        makeAxisKey(motorId),
        [consecutiveTimeouts](can_driver::SharedDriverState::AxisFeedbackState *feedback) {
            feedback->consecutiveTimeoutCount =
                static_cast<std::uint32_t>(consecutiveTimeouts);
            feedback->degraded = feedback->consecutiveTimeoutCount >=
                                 can_driver::kDefaultFeedbackDegradedTimeoutThreshold;
        });
}

void MtCan::resetReadTracking()
{
    std::lock_guard<std::mutex> lock(pendingReadMutex_);
    pendingReadRequests_.clear();
}

void MtCan::rememberSystemMotorId(MotorID motorId)
{
    std::lock_guard<std::mutex> lock(refreshMutex);
    systemMotorIdsByNodeId_[toMtProtocolNodeId(motorId)] = motorId;
}

MotorID MtCan::resolveSystemMotorId(uint8_t motorId) const
{
    std::lock_guard<std::mutex> lock(refreshMutex);
    const auto it = systemMotorIdsByNodeId_.find(motorId);
    return (it != systemMotorIdsByNodeId_.end()) ? it->second
                                                 : motorIdFromMtProtocolNodeId(motorId);
}

uint16_t MtCan::pendingReadKey(uint8_t motorId, uint8_t command)
{
    return static_cast<uint16_t>((static_cast<uint16_t>(motorId) << 8) | command);
}

std::chrono::milliseconds MtCan::computeReadRequestTimeout() const
{
    std::size_t motorCount = 1;
    {
        std::lock_guard<std::mutex> lock(refreshMutex);
        motorCount = std::max<std::size_t>(1, refreshMotorIds.size());
    }
    const auto refreshSleep = computeRefreshSleep(motorCount);
    const auto timeoutCycles = std::min<int64_t>(
        kMaxReadTimeoutCycles,
        kBaseReadTimeoutCycles + static_cast<int64_t>(motorCount));
    const auto timeout = refreshSleep * timeoutCycles;
    return std::max(kMinReadRequestTimeout, std::min(timeout, kMaxReadRequestTimeout));
}

std::chrono::milliseconds MtCan::computeTimeoutBackoff(std::size_t consecutiveTimeouts,
                                                       std::chrono::milliseconds baseTimeout)
{
    const std::size_t cappedTimeouts = std::min<std::size_t>(consecutiveTimeouts, 4);
    const auto multiplier = static_cast<int64_t>(1ULL << cappedTimeouts);
    const auto backoff = baseTimeout * multiplier;
    return std::min(std::chrono::milliseconds(500), std::max(baseTimeout, backoff));
}

// [FIX #1] 重写 handleResponse，修正 nodeId 提取
void MtCan::handleResponse(const CanTransport::Frame &frame)
{
    if (shuttingDown_.load(std::memory_order_acquire)) {
        return;
    }
    if (frame.isExtended) {
        return;
    }

    const uint16_t canId = static_cast<uint16_t>(frame.id & 0x7FF);
    if (canId < kResponseBaseId || canId >= kResponseBaseId + 0x100) {
        return; // 非本驱动响应帧，静默忽略（避免多设备总线日志洪泛）
    }

    if (frame.dlc == 0) {
        return;
    }

    const uint8_t command = frame.data[0];

    // [FIX #1] 用减法提取电机 ID，而非位掩码
    //   原代码: canId & 0xFF → 0x241 & 0xFF = 0x41 = 65（错误）
    //   修正后: canId - 0x240 → 0x241 - 0x240 = 1（正确）
    const uint8_t nodeId = static_cast<uint8_t>(canId - kResponseBaseId);

    switch (command) {
    case 0x9C:
    case 0x9A:
    case 0x92:
        markReadResponseReceived(nodeId, command);
        break;
    default:
        break;
    }

    bool shouldResetAfterZero = false;
    bool sharedFeedbackUpdated = false;
    MotorState sharedFeedbackSnapshot;

    {
        std::lock_guard<std::mutex> stateLock(stateMutex);
        MotorState &state = motorStates[nodeId];

        switch (command) {

        // ── 读取电机状态2应答 (0x9C) ──────────
        case 0x9C: {
            // [FIX #6] 完整解析: 温度、电流、速度、编码器位置
            if (frame.dlc >= 8) {
                state.temperature = static_cast<int8_t>(frame.data[1]);
                const int16_t rawCurrent = readInt16LE(frame, 2);
                state.current = static_cast<double>(rawCurrent) / 100.0;
                // 速度 int16_t, 单位 1 dps/LSB（保持协议原始单位）
                state.velocity = readInt16LE(frame, 4);
                state.encoderPosition = readUInt16LE(frame, 6);
                state.currentReceived = true;
                state.velocityReceived = true;
            }
            break;
        }

        // ── 读取电机状态1和错误标志应答 (0x9A) ──
        case 0x9A: {
            if (frame.dlc >= 8) {
                state.temperature = static_cast<int8_t>(frame.data[1]);
                state.voltageRaw1 = readUInt16LE(frame, 2);
                state.voltageRaw2 = readUInt16LE(frame, 4);
                const uint16_t errorCode = readUInt16LE(frame, 6);
                state.error = errorCode != 0;
                state.faultReceived = true;
                if (state.error) {
                    std::cerr << "[MtCan] Motor " << static_cast<int>(nodeId)
                              << " error code 0x" << std::hex << errorCode
                              << std::dec << '\n';
                }
            }
            break;
        }

        // ── 多圈角度应答 (0x92) ────────────────
        case 0x92: {
            // DATA[4~7] = int32_t LE, 单位 0.01°/LSB
            if (frame.dlc >= 8) {
                state.multiTurnAngle = readInt32LE(frame, 4);
                state.positionReceived = true;
            }
            break;
        }

        // ── 运动命令及开关命令应答（共用状态格式）──
        case 0xA1: // 转矩闭环
        case 0xA2: // 速度闭环
        case 0xA4: // 绝对位置
        case 0xA6: // 单圈位置
        case 0xA8: // 增量位置
        case 0xA9: // 力位混合控制
        case 0x80: // Motor Off
        case 0x81: // Motor Stop
        {
            // 应答格式与 0x9C 一致: temp(1), iq(2), speed(2), encoder(2)
            if (frame.dlc >= 8) {
                state.temperature = static_cast<int8_t>(frame.data[1]);
                const int16_t rawCurrent = readInt16LE(frame, 2);
                state.current = static_cast<double>(rawCurrent) / 100.0;
                state.velocity = readInt16LE(frame, 4);
                state.encoderPosition = readUInt16LE(frame, 6);
                state.currentReceived = true;
                state.velocityReceived = true;
            }
            break;
        }

        // ── 设置零点应答 (0x64) ──────────────
        case 0x64:
            shouldResetAfterZero = true;
            break;

        // ── 通讯中断保护设置应答 (0xB3) ────────
        case 0xB3: {
            if (frame.dlc >= 8) {
                const uint32_t confirmedTimeout =
                    static_cast<uint32_t>(frame.data[4]) |
                    (static_cast<uint32_t>(frame.data[5]) << 8) |
                    (static_cast<uint32_t>(frame.data[6]) << 16) |
                    (static_cast<uint32_t>(frame.data[7]) << 24);
                std::cout << "[MtCan] Motor " << static_cast<int>(nodeId)
                          << " communication timeout confirmed: "
                          << confirmedTimeout << " ms\n";
            }
            break;
        }

        // ── 加减速度设置应答 (0x43) ──────────
        case 0x43: {
            if (frame.dlc >= 8) {
                const uint8_t accelIndex = frame.data[1];
                const uint32_t confirmedAccel =
                    static_cast<uint32_t>(frame.data[4]) |
                    (static_cast<uint32_t>(frame.data[5]) << 8) |
                    (static_cast<uint32_t>(frame.data[6]) << 16) |
                    (static_cast<uint32_t>(frame.data[7]) << 24);
                const char *names[] = {"pos_accel", "pos_decel", "spd_accel", "spd_decel"};
                const char *name = (accelIndex <= 3) ? names[accelIndex] : "unknown";
                std::cout << "[MtCan] Motor " << static_cast<int>(nodeId)
                          << " " << name << " confirmed: "
                          << confirmedAccel << " dps/s\n";
            }
            break;
        }

        default:
            break;
        }

        sharedFeedbackSnapshot = state;
        switch (command) {
        case 0x9C:
        case 0x9A:
        case 0x92:
        case 0xA1:
        case 0xA2:
        case 0xA4:
        case 0xA6:
        case 0xA8:
        case 0xA9:
        case 0x80:
        case 0x81:
            sharedFeedbackUpdated = true;
            break;
        default:
            break;
        }
    }

    if (sharedFeedbackUpdated) {
        syncSharedFeedback(nodeId, sharedFeedbackSnapshot);
    }

    // 设置零点后需要系统复位才能生效（在锁外调用，避免死锁）
    if (shouldResetAfterZero) {
        resetSystem(nodeId);
    }
}
