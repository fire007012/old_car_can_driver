#ifndef MTCAN_H
#define MTCAN_H
#include "CanProtocol.h"
#include "can_driver/CanTransport.h"
#include "can_driver/RefreshScheduler.h"
#include "can_driver/SharedDriverState.h"
#include "can_driver/CanTxDispatcher.h"
#include <array>
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <vector>

class MtCan : public CanProtocol {
    friend class MtCanTestAccessor;

public:
    /**
     * @brief 构造函数
        * @param controller 基于 CanTransport 的 CAN 传输实现
     */
    MtCan(std::shared_ptr<CanTransport> controller,
          std::shared_ptr<CanTxDispatcher> txDispatcher);
    MtCan(std::shared_ptr<CanTransport> controller,
          std::shared_ptr<CanTxDispatcher> txDispatcher,
          std::shared_ptr<can_driver::SharedDriverState> sharedState,
          std::string deviceName);

    ~MtCan();

    /**
     * @brief 设置工作模式（目前仅记录状态，不发送额外命令）
     */
    bool setMode(MotorID motorId, MotorMode mode) override;

    /**
     * @brief 设置目标速度（命令 0xA2）
     */
    bool setVelocity(MotorID motorId, int32_t velocity) override;

    /**
     * @brief 设置加速度（映射到 0x43 速度规划加速度）
     */
    bool setAcceleration(MotorID motorId, int32_t acceleration) override;

    /**
     * @brief 设置减速度（映射到 0x43 速度规划减速度）
     */
    bool setDeceleration(MotorID motorId, int32_t deceleration) override;

    /**
     * @brief 设置通讯中断保护时间（0xB3）
     */
    bool setCommunicationTimeout(uint32_t timeoutMs);

    /**
     * @brief 设置速度规划加速度（0x43, index=0x02）
     */
    bool setSpeedAcceleration(MotorID id, uint32_t accelDpsPerSec);

    /**
     * @brief 设置速度规划减速度（0x43, index=0x03）
     */
    bool setSpeedDeceleration(MotorID id, uint32_t decelDpsPerSec);

    /**
     * @brief 设置位置规划加速度（0x43, index=0x00）
     */
    bool setPositionAcceleration(MotorID id, uint32_t accelDpsPerSec);

    /**
     * @brief 设置位置规划减速度（0x43, index=0x01）
     */
    bool setPositionDeceleration(MotorID id, uint32_t decelDpsPerSec);

    /**
     * @brief 位置控制命令（0xA4），同时携带最大速度
     */
    bool setPosition(MotorID motorId, int32_t position) override;

    /**
     * @brief 快写位置命令（用于 CSP 模式）
     * @note MtCan 协议暂不支持 CSP 模式，此接口仅为满足基类要求
     */
    bool quickSetPosition(MotorID motorId, int32_t position) override;

    /**
     * @brief 标记电机进入可控状态（协议无独立使能命令）
     */
    bool Enable(MotorID motorId) override;

    /**
     * @brief 关闭电机输出（0x80, Motor Off）
     */
    bool Disable(MotorID motorId) override;

    /**
     * @brief 停止（0x81）
     */
    bool Stop(MotorID motorId) override;
    bool ResetFault(MotorID motorId) override;

    /**
     * @brief 返回缓存的电机位置
     */
    int64_t getPosition(MotorID motorId) const override;

    /**
     * @brief 返回电流（0x9C 返回的值，单位 0.01A）
     */
    int16_t getCurrent(MotorID motorId) const override;

    /**
     * @brief 返回电机速度（0x9C 返回的值，单位 1 dps/LSB）
     */
    int32_t getVelocity(MotorID motorId) const override;
    bool isEnabled(MotorID motorId) const override;
    bool hasFault(MotorID motorId) const override;

    /**
     * @brief 配置需轮询的电机并启动 1ms 刷新任务
     */
    void initializeMotorRefresh(const std::vector<MotorID> &motorIds) override;

    /// 设置状态轮询频率（Hz）；<=0 表示使用默认自适应周期。
    void setRefreshRateHz(double hz);
    /// 设置电机注册后下发的通讯中断保护时间（ms）。
    void setCommunicationTimeoutOnInit(uint32_t timeoutMs);
    /// 返回当前注册电机的建议查询周期。
    std::chrono::milliseconds refreshSleepInterval() const;
    /// 返回单个 MT 读请求在判定超时前应等待的时间。
    std::chrono::milliseconds readResponseTimeout() const;
    using RefreshQuery = can_driver::MtRefreshQuery;
    bool issueRefreshQuery(MotorID motorId, RefreshQuery query);

private:
    struct MotorState {
        int32_t position = 0;
        int64_t multiTurnAngle = 0;  ///< 0x92 多圈角度，单位 0.01°
        int16_t velocity = 0;        ///< 速度，单位 1 dps/LSB（协议原始单位）
        double current = 0.0;
        int32_t commandedVelocity = 0;
        int8_t temperature = 0;      ///< 温度（°C）
        uint16_t voltageRaw1 = 0;    ///< 0x9A DATA[2..3]（电压/保留，协议相关）
        uint16_t voltageRaw2 = 0;    ///< 0x9A DATA[4..5]（电压/保留，协议相关）
        uint16_t encoderPosition = 0; ///< 单圈编码器位置
        bool enabled = false;
        bool error = false;
        bool positionReceived = false;
        bool velocityReceived = false;
        bool currentReceived = false;
        bool modeReceived = false;
        bool enabledReceived = false;
        bool faultReceived = false;
        MotorMode mode = MotorMode::Velocity;
    };
    struct PendingReadRequest {
        std::chrono::steady_clock::time_point lastSent {};
        std::chrono::steady_clock::time_point nextEligibleSend {};
        bool queued {false};
        bool inFlight {false};
        std::size_t consecutiveTimeouts {0};
    };

    std::shared_ptr<CanTransport> canController;
    std::shared_ptr<CanTxDispatcher> txDispatcher_;
    std::shared_ptr<can_driver::SharedDriverState> sharedState_;
    std::string deviceName_;
    mutable std::unordered_map<uint8_t, MotorState> motorStates;
    mutable std::mutex stateMutex;
    std::size_t receiveHandlerId = 0;
    std::vector<uint8_t> refreshMotorIds;
    mutable std::unordered_map<uint8_t, MotorID> systemMotorIdsByNodeId_;
    mutable std::mutex refreshMutex;
    std::atomic<double> refreshRateHz_{0.0};
    std::atomic<bool> shuttingDown_{false};
    mutable std::mutex pendingReadMutex_;
    std::unordered_map<uint16_t, PendingReadRequest> pendingReadRequests_;
    std::atomic<uint32_t> communicationTimeoutOnInitMs_{0};

    /**
     * @brief 将节点 ID 组合成 CAN ID（高位取 canBaseId，高 8 位 + motorId）
     */
    uint16_t encodeSendCanId(uint8_t motorId) const;
    /**
     * @brief 发送标准 8 字节 CAN 帧
     */
    void sendFrame(uint16_t canId, uint8_t command, const std::array<uint8_t, 4> &payload) const;
    /**
     * @brief 触发读状态命令（0x9C）
     */
    bool requestState(uint8_t motorId);
    /**
     * @brief 触发读错误命令（0x9A）
     */
    bool requestError(uint8_t motorId);
    /**
     * @brief 触发读多圈角度命令（0x92）
     */
    bool requestMultiTurnAngle(uint8_t motorId);
    /**
     * @brief 复位系统（0x76）
     */
    void resetSystem(uint8_t motorId) const;
    /**
     * @brief 设置零点（0x64）
     */
    void setZeroPosition(uint8_t motorId) const;
    void stopRefreshLoop();
    /**
     * @brief 通用加减速写入（0x43）
     */
    bool writeAcceleration(uint8_t motorId, uint8_t index, uint32_t value);
    /**
     * @brief 广播通讯超时保护给所有已注册电机
     */
    void broadcastCommunicationTimeout(uint32_t timeoutMs);
    std::chrono::milliseconds computeRefreshSleep(std::size_t motorCount) const;
    std::chrono::milliseconds computeReadRequestTimeout() const;
    /**
     * @brief 解析 CAN 返回帧，更新缓存
     */
    void handleResponse(const CanTransport::Frame &data);
    bool submitTx(const CanTransport::Frame &frame,
                  CanTxDispatcher::Category category,
                  const char *source) const;
    static CanTransport::Frame makeCommandFrame(uint16_t canId,
                                                uint8_t command,
                                                const std::array<uint8_t, 4> &payload);
    void onReadDispatchResult(uint8_t motorId,
                              uint8_t command,
                              bool attemptedSend,
                              CanTransport::SendResult sendResult,
                              std::chrono::steady_clock::time_point eventTime);
    bool tryIssueReadCommand(uint8_t motorId, uint8_t command);
    void markReadResponseReceived(uint8_t motorId, uint8_t command);
    void resetReadTracking();
    void rememberSystemMotorId(MotorID motorId);
    MotorID resolveSystemMotorId(uint8_t motorId) const;
    can_driver::SharedDriverState::AxisKey makeAxisKey(uint8_t motorId) const;
    void syncSharedFeedback(uint8_t motorId, const MotorState &state) const;
    void syncSharedCommand(uint8_t motorId,
                           int64_t targetPosition,
                           int32_t targetVelocity,
                           MotorMode desiredMode,
                           bool valid) const;
    void syncSharedModeSelection(uint8_t motorId, MotorMode desiredMode) const;
    void syncSharedIntent(uint8_t motorId, can_driver::AxisIntent intent) const;
    void noteSharedTimeout(uint8_t motorId, std::size_t consecutiveTimeouts) const;
    static uint16_t pendingReadKey(uint8_t motorId, uint8_t command);
    static std::chrono::milliseconds computeTimeoutBackoff(std::size_t consecutiveTimeouts,
                                                           std::chrono::milliseconds baseTimeout);
};

#endif // MTCAN_H
