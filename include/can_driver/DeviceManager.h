#ifndef CAN_DRIVER_DEVICE_MANAGER_H
#define CAN_DRIVER_DEVICE_MANAGER_H

#include "can_driver/CanProtocol.h"
#include "can_driver/CanTxDispatcher.h"
#include "can_driver/CanType.h"
#include "can_driver/DeviceRefreshWorker.h"
#include "can_driver/DamiaoCan.h"
#include "can_driver/EyouCan.h"
#include "can_driver/IDeviceManager.h"
#include "can_driver/InnfosEcbProtocol.h"
#include "can_driver/MotorID.h"
#include "can_driver/MtCan.h"
#include "can_driver/SharedDriverState.h"
#include "can_driver/SocketCanController.h"
#include "can_driver/UdpCanTransport.h"

#include <atomic>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

/**
 * @brief 管理 CAN 设备、传输层和协议实例的集中入口。
 *
 * 一个 device（如 can0/vcan0）只会持有一个传输层实例，
 * 但可以按需同时挂载 MT/PP 两套协议对象。
 */
class DeviceManager : public IDeviceManager {
    friend class DeviceManagerTestAccessor;
public:
    /// 确保底层传输已建立。重复调用是幂等的。
    bool ensureTransport(const std::string &device, bool loopback = false) override;
    /// 在指定设备上确保目标协议实例存在（要求 transport 已创建）。
    bool ensureProtocol(const std::string &device, CanType type) override;
    /// 按设备完成 transport/protocol 初始化，并触发首轮状态刷新。
    bool initDevice(const std::string &device,
                    const std::vector<std::pair<CanType, MotorID>> &motors,
                    bool loopback = false) override;
    /// 为协议注册需要周期刷新状态的电机列表。
    void startRefresh(const std::string &device,
                      CanType type,
                      const std::vector<MotorID> &ids) override;
    /// 设置所有协议实例的状态轮询频率（Hz）；<=0 恢复协议默认策略。
    void setRefreshRateHz(double hz) override;
    /// 设置指定 device 的状态轮询频率（Hz）；<=0 回退到全局默认策略。
    void setDeviceRefreshRateHz(const std::string &device, double hz) override;
    /// 设置 MT 协议在注册电机后下发的通讯中断保护时间（ms）。
    void setMtCommunicationTimeoutOnInitMs(uint32_t timeoutMs) override;
    /// 设置 PP 协议是否使用快写命令（CMD=0x05）。
    void setPpFastWriteEnabled(bool enabled) override;
    /// 兼容旧接口：同时设置 PP 位置/CSP 命令默认预配置速度（0x09，协议原始单位）。
    void setPpDefaultPositionVelocityRaw(int32_t velocityRaw) override;
    /// 设置 PP 位置模式命令默认预配置速度（0x09，协议原始单位）。
    void setPpPositionDefaultVelocityRaw(int32_t velocityRaw) override;
    /// 设置 PP CSP 模式命令默认预配置速度（0x09，协议原始单位）。
    void setPpCspDefaultVelocityRaw(int32_t velocityRaw) override;
    /// 停止并释放单个设备资源，不影响其他 device。
    void shutdownDevice(const std::string &device) override;
    /// 停止并释放所有设备资源。
    void shutdownAll() override;

    /// 读取协议实例（不存在返回 nullptr）。
    std::shared_ptr<CanProtocol> getProtocol(const std::string &device, CanType type) const override;
    /// 返回该设备命令互斥锁（不存在返回 nullptr）。
    std::shared_ptr<std::mutex> getDeviceMutex(const std::string &device) const override;
    /// 返回设备是否 ready。
    bool isDeviceReady(const std::string &device) const override;
    std::shared_ptr<can_driver::SharedDriverState> getSharedDriverState() const override;
    /// 返回传输层实例（不存在返回 nullptr）。
    std::shared_ptr<SocketCanController> getTransport(const std::string &device) const;
    /// 当前已初始化的 transport 数量。
    std::size_t deviceCount() const override;

private:
    bool isUdpDevice(const std::string &device) const;
    bool isEcbDevice(const std::string &device) const;
    std::shared_ptr<CanTransport> getTransportBaseLocked(const std::string &device) const;
    bool shutdownTransportLocked(const std::string &device);
    bool initializeTransportLocked(const std::string &device, bool loopback);

    struct DeviceRefreshRuntime {
        std::string deviceName;
        std::weak_ptr<can_driver::SharedDriverState> sharedState;
        std::weak_ptr<MtCan> mtProtocol;
        std::weak_ptr<EyouCan> ppProtocol;
        std::weak_ptr<DamiaoCan> dmProtocol;
        std::shared_ptr<can_driver::DeviceRefreshWorker> worker;
        std::atomic<bool> mtActive{false};
        std::atomic<bool> ppActive{false};
        std::atomic<bool> dmActive{false};
        std::vector<std::uint8_t> mtMotorIds;
        std::vector<std::uint8_t> ppMotorIds;
        std::vector<std::uint8_t> dmMotorIds;
        std::unordered_map<std::uint8_t, can_driver::PpAxisRefreshScheduleState> ppScheduleStates;
        std::unordered_map<std::uint8_t, can_driver::DmAxisRefreshScheduleState> dmScheduleStates;
        std::uint64_t refreshCycleCount{0};
        std::uint64_t mtScheduleCycleCount{0};
        std::uint64_t ppScheduleCycleCount{0};
        std::uint64_t lastObservedTxBackpressure{0};
        std::uint64_t queryPressureUntilCycle{0};
        std::chrono::steady_clock::time_point nextMtTick {};
        std::chrono::steady_clock::time_point nextPpTick {};
        std::chrono::steady_clock::time_point nextDmTick {};
        mutable std::mutex scheduleMutex;
    };

    void syncDeviceRefreshRuntimeLocked(const std::string &device);
    void startDeviceRefreshWorkerLocked(const std::string &device);
    void stopDeviceRefreshWorkerLocked(const std::string &device);
    void stopAllDeviceRefreshWorkersLocked();
    void resetDeviceRuntimeLocked(const std::string &device);
    void shutdownDeviceLocked(const std::string &device);
    double effectiveRefreshRateHzLocked(const std::string &device) const;
    void applyRefreshRateLocked(const std::string &device, double hz);
    static double normalizeRefreshRateHz(double hz);

    // 读多写少：读取协议/transport 时使用 shared_lock，创建/销毁时 unique_lock。
    mutable std::shared_mutex mutex_;
    // key = can device name（例如 can0/vcan0）。
    std::map<std::string, std::shared_ptr<SocketCanController>> transports_;
    std::map<std::string, std::shared_ptr<UdpCanTransport>> udpTransports_;
    std::map<std::string, std::shared_ptr<CanTxDispatcher>> txDispatchers_;
    std::map<std::string, std::shared_ptr<MtCan>> mtProtocols_;
    std::map<std::string, std::shared_ptr<EyouCan>> eyouProtocols_;
    std::map<std::string, std::shared_ptr<DamiaoCan>> damiaoProtocols_;
    std::map<std::string, std::shared_ptr<InnfosEcbProtocol>> ecbProtocols_;
    std::set<std::string> ecbDevices_;
    std::map<std::string, std::shared_ptr<DeviceRefreshRuntime>> deviceRefreshRuntimes_;
    std::shared_ptr<can_driver::SharedDriverState> sharedState_{
        std::make_shared<can_driver::SharedDriverState>()};
    bool ppFastWriteEnabled_{false};
    uint32_t mtCommunicationTimeoutOnInitMs_{0};
    int32_t ppPositionDefaultVelocityRaw_{EyouCan::kDefaultPositionVelocityRaw};
    int32_t ppCspDefaultVelocityRaw_{EyouCan::kDefaultPositionVelocityRaw};
    double refreshRateHz_{0.0};
    std::unordered_map<std::string, double> deviceRefreshRateOverrides_;
    // 每个设备一把命令互斥锁，避免多个控制线程并发下发命令时互相打断。
    std::map<std::string, std::shared_ptr<std::mutex>> deviceCmdMutexes_;
};

#endif // CAN_DRIVER_DEVICE_MANAGER_H
