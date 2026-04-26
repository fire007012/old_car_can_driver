#ifndef CAN_DRIVER_CAN_DRIVER_HW_H
#define CAN_DRIVER_CAN_DRIVER_HW_H

#include "can_driver/CanProtocol.h"
#include "can_driver/CanDriverHwTypes.h"
#include "can_driver/CanDriverRuntime.h"
#include "can_driver/CanType.h"
#include "can_driver/command_gate.hpp"
#include "can_driver/DeviceManager.h"
#include "can_driver/IDeviceManager.h"
#include "can_driver/JointConfigParser.h"
#include "can_driver/lifecycle_driver_ops.hpp"
#include "can_driver/MotorID.h"
#include "can_driver/motor_action_executor.hpp"
#include "can_driver/motor_maintenance_service.hpp"
#include "can_driver/operational_coordinator.hpp"

#include <can_driver/MotorCommand.h>
#include <can_driver/MotorState.h>
#include <can_driver/SetZeroLimit.h>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <urdf/model.h>

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <atomic>
#include <deque>
#include <functional>
#include <filesystem>

class DriverRosEndpoints;

/**
 * @brief hardware_interface::RobotHW 实现，将 MtCan/EyouCan 协议层桥接到 ros_control。
 *
 * 每个 joint 在 YAML 中独立指定 can_device 和 protocol，节点按需创建传输实例：
 *   - 同一 can_device 上的所有 joint 共享一个 SocketCanController
 *   - 同一 can_device 上的 MT/PP joint 分别共享一个 MtCan / EyouCan 实例
 */
class CanDriverHW : public hardware_interface::RobotHW {
public:
    struct InitOptions {
        bool enable_ros_endpoints{true};
        bool warn_on_duplicate_motor_ids{true};
    };

    struct DirectCommandEndpoint {
        std::string jointName;
        std::size_t jointIndex{0};
    };

    struct JointRuntimeStateView {
        std::string jointName;
        std::string controlMode;
        double position{0.0};
        double velocity{0.0};
        double effort{0.0};
        bool deviceReady{false};
        bool enabled{false};
        bool fault{false};
        bool feedbackFresh{false};
        bool commandValid{false};
    };

    CanDriverHW();
    explicit CanDriverHW(std::shared_ptr<IDeviceManager> deviceManager);
    ~CanDriverHW() override;

    /**
     * @brief 从 rosparam 读取 joints 配置，创建传输/协议实例，注册 ros_control 接口。
     * @param nh   节点 NodeHandle（保留给公共命名空间扩展）
     * @param pnh  私有 NodeHandle（读取参数并发布 ~/ 接口）
     */
    bool init(ros::NodeHandle &nh, ros::NodeHandle &pnh);
    bool init(ros::NodeHandle &nh, ros::NodeHandle &pnh, const InitOptions &options);

    /**
     * @brief 从协议缓存拉取最新 pos/vel/eff，写入 ros_control 状态缓存。
     */
    void read(const ros::Time &time, const ros::Duration &period) override;

    /**
     * @brief 从 ros_control 命令缓存读取目标值，下发给协议层。
     */
    void write(const ros::Time &time, const ros::Duration &period) override;

    can_driver::SystemOpMode lifecycleMode() const
    {
        return lifecycleCoordinator_.mode();
    }

    can_driver::OperationalCoordinator &operationalCoordinator()
    {
        return lifecycleCoordinator_;
    }

    // 供 DriverRosEndpoints 使用的 ROS 装配辅助。
    void configureMotorMaintenanceService(MotorMaintenanceService &service);
    std::vector<DirectCommandEndpoint> directCommandEndpoints() const;
    std::vector<JointRuntimeStateView> snapshotJointRuntimeStates() const;
    void acceptDirectCommand(std::size_t jointIndex,
                             bool isVelocity,
                             double value,
                             const ros::Time &stamp);
    bool lookupJointByMotorId(uint16_t motorId, can_driver::CanDriverJointConfig *joint) const;
    void clearDirectCommand(const std::string &jointName);
    bool commitModeSwitch(uint16_t motorId, can_driver::AxisControlMode mode);
    bool getZeroOffset(uint16_t motorId, double* zeroOffset) const;
    bool commitZero(uint16_t motorId,
                    double zeroOffset,
                    double previousZeroOffset);
    bool commitLimits(uint16_t motorId,
                      double baseMin,
                      double baseMax,
                      double zeroOffset);
    void publishMotorStates(ros::Publisher &publisher);
    void publishLifecycleState(ros::Publisher &publisher);
    double motorStatePeriodSec() const
    {
        return statePublishPeriodSec_;
    }
    int directCommandQueueSize() const
    {
        return directCmdQueueSize_;
    }

private:
    // -----------------------------------------------------------------------
    // 关节配置
    // -----------------------------------------------------------------------
    using JointConfig = can_driver::CanDriverJointConfig;
    using DeviceProtocolGroup = can_driver::CanDriverDeviceProtocolGroup;
    std::deque<JointConfig> joints_;
    std::map<std::string, std::size_t> jointIndexByName_;
    std::vector<DeviceProtocolGroup> jointGroups_;
    std::vector<int32_t>             rawCommandBuffer_;
    std::vector<uint8_t>             commandValidBuffer_;
    std::vector<can_driver::CanDriverPreparedCommand> preparedCommandBuffer_;
    std::map<uint16_t, double>       jointZeroOffsetRadByMotorId_;

    can_driver::CanDriverRuntime runtime_;
    std::shared_ptr<IDeviceManager> &deviceManager_;
    MotorActionExecutor &motorActionExecutor_;
    can_driver::LifecycleDriverOps &lifecycleDriverOps_;
    CommandGate &commandGate_;
    std::atomic<bool> &active_;
    can_driver::OperationalCoordinator &lifecycleCoordinator_;
    std::map<std::string, bool> &deviceLoopbackByName_;

    // -----------------------------------------------------------------------
    // ros_control 接口对象
    // -----------------------------------------------------------------------
    hardware_interface::JointStateInterface    jntStateIface_;
    hardware_interface::VelocityJointInterface velIface_;
    hardware_interface::PositionJointInterface posIface_;

    // 关节限位接口（从 URDF 和 rosparam 读取限位，自动钳制命令值）
    joint_limits_interface::PositionJointSaturationInterface posLimitsIface_;
    joint_limits_interface::VelocityJointSaturationInterface velLimitsIface_;

    std::unique_ptr<DriverRosEndpoints> rosEndpoints_;

    // 生命周期与并发控制
    mutable std::mutex        jointStateMutex_;
    double directCmdTimeoutSec_{0.5};
    double statePublishPeriodSec_{0.1};
    double motorQueryHz_{0.0};
    int directCmdQueueSize_{1};
    bool debugBypassRosControl_{false};
    bool warnOnDuplicateMotorIds_{true};
    bool ppFastWriteEnabled_{false};
    double ppPositionDefaultVelocityRadS_{0.0};
    double ppCspDefaultVelocityRadS_{0.0};
    double startupPositionSyncTimeoutSec_{1.0};
    double startupProbeQueryHz_{5.0};
    uint32_t mtCommunicationTimeoutOnInitMs_{0};
    double safetyFeedbackFreshnessTimeoutSec_{0.5};
    bool safetyStopOnFault_{true};
    bool safetyRequireEnabledForMotion_{true};
    bool lifecycleRequireEnabledForRunning_{true};
    double maxPositionStepRad_{0.0};
    bool safetyHoldAfterDeviceRecover_{true};

    // -----------------------------------------------------------------------
    // 内部辅助
    // -----------------------------------------------------------------------
    void resetInternalState();
    bool loadRuntimeParams(const ros::NodeHandle &pnh);
    bool parseAndSetupJoints(const ros::NodeHandle &pnh);
    void rebuildJointGroups();
    void registerJointInterfaces();
    void loadJointLimits(const ros::NodeHandle &pnh);
    bool syncStartupPositionAndCommands(const std::string &deviceFilter = std::string());
    bool applyPersistedPpZeroOffsets(const std::string &deviceFilter = std::string());
    bool applyInitialModes(const std::string &deviceFilter = std::string());
    bool applyPerAxisPpDefaultVelocities(const std::string &deviceFilter = std::string());
    void configureCommandGate();
    void configureLifecycleCoordinator();
    void holdCommandsForLifecycleTransition();
    std::vector<CommandGate::Snapshot> captureCommandSnapshots() const;
    void syncLifecycleTargets();
    MotorActionExecutor::Target makeMotorTarget(const JointConfig &jc) const;

    /**
     * @brief 初始化（或重新初始化）指定 CAN 通道。
     *        若 transport 尚不存在则创建；若已存在则先 shutdown 再重新 initialize。
     */
    bool initDevice(const std::string &device, bool loopback = false);

    /**
     * @brief 返回指定通道和协议类型对应的 CanProtocol 指针，不存在时返回 nullptr。
     */
    std::shared_ptr<CanProtocol> getProtocol(const std::string &device, CanType type) const;
    std::shared_ptr<std::mutex> getDeviceMutex(const std::string &device) const;
    bool isDeviceReady(const std::string &device) const;
    bool getFreshAxisFeedback(const JointConfig &joint,
                              can_driver::SharedDriverState::AxisFeedbackState *feedback) const;
    bool requireAxisDisabledForConfiguration(const JointConfig &joint,
                                             const char *operation,
                                             std::string *message) const;
    bool lifecycleHealthHealthy(std::string *detail) const;
    std::string defaultLocalZeroOffsetFilePath() const;
    bool loadPersistedLocalZeroOffsets();
    bool savePersistedLocalZeroOffsets() const;

    bool ppLocalZeroOffsetPersistenceEnabled_{false};
    bool ppZeroOffsetPersistOnApplyToMotor_{true};
    std::string ppLocalZeroOffsetFilePath_;
};

#endif // CAN_DRIVER_CAN_DRIVER_HW_H
