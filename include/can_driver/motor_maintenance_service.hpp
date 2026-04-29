#ifndef CAN_DRIVER_MOTOR_MAINTENANCE_SERVICE_HPP
#define CAN_DRIVER_MOTOR_MAINTENANCE_SERVICE_HPP

#include "can_driver/CanDriverHwTypes.h"
#include "can_driver/CanProtocol.h"
#include "can_driver/AxisCommandSemantics.h"
#include "can_driver/motor_action_executor.hpp"
#include "can_driver/SharedDriverState.h"

#include <can_driver/ApplyLimits.h>
#include <can_driver/MotorCommand.h>
#include <can_driver/SetZero.h>
#include <can_driver/SetZeroLimit.h>

#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <ros/ros.h>

class MotorMaintenanceService {
public:
    struct AdvertiseOptions {
        bool motorCommand{true};
        bool setZero{true};
        bool applyLimits{true};
        bool setZeroLimit{true};
    };

    using JointConfig = can_driver::CanDriverJointConfig;
    using ActivityChecker = std::function<bool()>;
    using FreshFeedbackGetter = std::function<bool(
        const JointConfig &,
        can_driver::SharedDriverState::AxisFeedbackState *)>;
    using JointLookup = std::function<bool(uint16_t, JointConfig *)>;
    using DirectCommandClearer = std::function<void(const std::string &)>;
    using ModeSwitchCommitter =
        std::function<bool(uint16_t, can_driver::AxisControlMode)>;
    using ZeroCommitter =
        std::function<bool(uint16_t, double, double, bool)>;
    using ZeroOffsetGetter =
        std::function<bool(uint16_t, double*)>;
    using LimitCommitter =
        std::function<bool(uint16_t, double, double, double, bool)>;
    using DisabledRequirementChecker =
        std::function<bool(const JointConfig &, const char *, std::string *)>;
    using ProtocolGetter =
        std::function<std::shared_ptr<CanProtocol>(const std::string &, CanType)>;
    using DeviceMutexGetter =
        std::function<std::shared_ptr<std::mutex>(const std::string &)>;

    MotorMaintenanceService() = default;

    void configure(ActivityChecker isActive,
                   MotorActionExecutor *motorActionExecutor,
                   JointLookup lookupJointByMotorId,
                   DirectCommandClearer clearDirectCommand,
                   ModeSwitchCommitter commitModeSwitch,
                   ZeroCommitter commitZero,
                   ZeroOffsetGetter getZeroOffset,
                   LimitCommitter commitLimits,
                   FreshFeedbackGetter getFreshAxisFeedback,
                   DisabledRequirementChecker requireAxisDisabledForConfiguration,
                   ProtocolGetter getProtocol,
                   DeviceMutexGetter getDeviceMutex,
                   bool persistZeroOffsetOnApplyToMotor);

    void initialize(ros::NodeHandle &pnh);
    void initialize(ros::NodeHandle &pnh,
                    const AdvertiseOptions &options);
    void shutdown();
    bool SetModeByMotorId(uint16_t motorId,
                          double modeValue,
                          std::string *message);
    bool SetZeroByMotorId(uint16_t motorId,
                          double zeroOffsetRad,
                          bool useCurrentPositionAsZero,
                          bool applyToMotor,
                          double* previousZeroOffsetRad,
                          double* currentPositionRad,
                          double* appliedZeroOffsetRad,
                          std::string *message);
    bool ApplyLimitsByMotorId(uint16_t motorId,
                              double minPositionRad,
                              double maxPositionRad,
                              bool useUrdfLimits,
                              bool applyToMotor,
                              bool requireCurrentInsideLimits,
                              double* currentPositionRad,
                              double* activeZeroOffsetRad,
                              double* appliedMinRad,
                              double* appliedMaxRad,
                              std::string *message);

private:
    struct ResolvedLimits {
        double baseMin{0.0};
        double baseMax{0.0};
    };

    bool resolveBaseLimits(const JointConfig& target,
                           bool useUrdfLimits,
                           double requestedMinRad,
                           double requestedMaxRad,
                           ResolvedLimits* out,
                           std::string* message) const;
    bool resolveCurrentReportedPosition(const JointConfig& target,
                                        double* currentPositionRad,
                                        bool* hasFreshFeedback,
                                        std::string* message) const;
    bool resolveCurrentZeroOffset(const JointConfig& target,
                                  bool applyToMotor,
                                  double* zeroOffsetRad,
                                  std::string* message) const;
    bool handleSetModeRequest(const JointConfig &target,
                              uint16_t motorId,
                              double modeValue,
                              std::string *message);
    bool lookupJointByMotorId(uint16_t motorId, JointConfig *joint) const;
    MotorActionExecutor::Target makeMotorTarget(const JointConfig &joint) const;
    bool waitForPpModeConfirmation(const JointConfig &joint,
                                   CanProtocol::MotorMode expectedMode,
                                   std::string *message) const;

    bool onMotorCommand(can_driver::MotorCommand::Request &req,
                        can_driver::MotorCommand::Response &res);
    bool onSetZero(can_driver::SetZero::Request &req,
                   can_driver::SetZero::Response &res);
    bool onApplyLimits(can_driver::ApplyLimits::Request &req,
                       can_driver::ApplyLimits::Response &res);
    bool onSetZeroLimit(can_driver::SetZeroLimit::Request &req,
                        can_driver::SetZeroLimit::Response &res);

    ActivityChecker isActive_;
    MotorActionExecutor *motorActionExecutor_{nullptr};
    JointLookup lookupJointByMotorId_;
    DirectCommandClearer clearDirectCommand_;
    ModeSwitchCommitter commitModeSwitch_;
    ZeroCommitter commitZero_;
    ZeroOffsetGetter getZeroOffset_;
    LimitCommitter commitLimits_;
    FreshFeedbackGetter getFreshAxisFeedback_;
    DisabledRequirementChecker requireAxisDisabledForConfiguration_;
    ProtocolGetter getProtocol_;
    DeviceMutexGetter getDeviceMutex_;
    bool persistZeroOffsetOnApplyToMotor_{true};

    ros::ServiceServer motorCmdSrv_;
    ros::ServiceServer setZeroSrv_;
    ros::ServiceServer applyLimitsSrv_;
    ros::ServiceServer setZeroLimitSrv_;
};

#endif // CAN_DRIVER_MOTOR_MAINTENANCE_SERVICE_HPP
