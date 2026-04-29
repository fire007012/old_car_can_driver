#ifndef CAN_DRIVER_AXIS_COMMAND_SEMANTICS_H
#define CAN_DRIVER_AXIS_COMMAND_SEMANTICS_H

#include "can_driver/CanDriverHwTypes.h"
#include "can_driver/CanProtocol.h"

#include <algorithm>
#include <cmath>
#include <string>

namespace can_driver {

enum class AxisControlMode {
    Position,
    Velocity,
    Csp,
    Unknown,
};

inline AxisControlMode axisControlModeFromString(const std::string &controlMode)
{
    if (controlMode == "velocity") {
        return AxisControlMode::Velocity;
    }
    if (controlMode == "csp") {
        return AxisControlMode::Csp;
    }
    if (controlMode == "position") {
        return AxisControlMode::Position;
    }
    return AxisControlMode::Unknown;
}

inline const char *axisControlModeName(AxisControlMode mode)
{
    switch (mode) {
    case AxisControlMode::Position:
        return "position";
    case AxisControlMode::Velocity:
        return "velocity";
    case AxisControlMode::Csp:
        return "csp";
    case AxisControlMode::Unknown:
        return "unknown";
    }
    return "unknown";
}

inline bool controlModeUsesVelocitySemantics(AxisControlMode mode)
{
    return mode == AxisControlMode::Velocity;
}

inline bool controlModeUsesVelocitySemantics(const std::string &controlMode)
{
    return controlModeUsesVelocitySemantics(axisControlModeFromString(controlMode));
}

inline bool controlModeUsesPositionSemantics(AxisControlMode mode)
{
    return !controlModeUsesVelocitySemantics(mode);
}

inline bool controlModeUsesPositionSemantics(const std::string &controlMode)
{
    return controlModeUsesPositionSemantics(axisControlModeFromString(controlMode));
}

inline PreparedCommandRoute controlModeDispatchRoute(AxisControlMode mode)
{
    if (mode == AxisControlMode::Velocity) {
        return PreparedCommandRoute::Velocity;
    }
    if (mode == AxisControlMode::Csp) {
        return PreparedCommandRoute::Csp;
    }
    return PreparedCommandRoute::Position;
}

inline PreparedCommandRoute controlModeDispatchRoute(const std::string &controlMode)
{
    return controlModeDispatchRoute(axisControlModeFromString(controlMode));
}

inline const char *controlModeSemanticLabel(AxisControlMode mode)
{
    return controlModeUsesVelocitySemantics(mode) ? "velocity" : "position";
}

inline bool controlModeHasDirectCommand(const CanDriverJointConfig &joint, AxisControlMode mode)
{
    return controlModeUsesVelocitySemantics(mode) ? joint.hasDirectVelCmd : joint.hasDirectPosCmd;
}

inline ros::Time controlModeLastDirectCommandTime(const CanDriverJointConfig &joint,
                                                  AxisControlMode mode)
{
    return controlModeUsesVelocitySemantics(mode) ? joint.lastDirectVelTime
                                                  : joint.lastDirectPosTime;
}

inline double controlModeDirectCommandValue(const CanDriverJointConfig &joint, AxisControlMode mode)
{
    return controlModeUsesVelocitySemantics(mode) ? joint.directVelCmd : joint.directPosCmd;
}

inline double controlModeFallbackCommandValue(const CanDriverJointConfig &joint,
                                              AxisControlMode mode)
{
    return controlModeUsesVelocitySemantics(mode) ? joint.velCmd : joint.posCmd;
}

inline double controlModeSelectedCommandValue(const CanDriverJointConfig &joint,
                                              AxisControlMode mode,
                                              bool *usesDirect = nullptr)
{
    const bool direct = controlModeHasDirectCommand(joint, mode);
    if (usesDirect != nullptr) {
        *usesDirect = direct;
    }
    return direct ? controlModeDirectCommandValue(joint, mode)
                  : controlModeFallbackCommandValue(joint, mode);
}

inline double controlModeActualFeedbackValue(const CanDriverJointConfig &joint, AxisControlMode mode)
{
    return controlModeUsesVelocitySemantics(mode) ? joint.vel : joint.pos;
}

inline double effectivePositionScale(const CanDriverJointConfig &joint)
{
    return joint.positionScale * joint.directionSign;
}

inline double effectiveVelocityScale(const CanDriverJointConfig &joint)
{
    return joint.velocityScale * joint.directionSign;
}

inline double rawPositionToJointPosition(const CanDriverJointConfig &joint, double rawPosition)
{
    return rawPosition * effectivePositionScale(joint) + joint.zeroOffsetRad;
}

inline double jointPositionToRawPositionCommand(const CanDriverJointConfig &joint,
                                               double jointPosition)
{
    return jointPosition - joint.zeroOffsetRad;
}

inline double controlModeScale(const CanDriverJointConfig &joint, AxisControlMode mode)
{
    return controlModeUsesVelocitySemantics(mode) ? effectiveVelocityScale(joint)
                                                  : effectivePositionScale(joint);
}

inline double controlModeAlignmentTolerance(const CanDriverJointConfig &joint,
                                            AxisControlMode mode)
{
    return std::max(std::fabs(controlModeScale(joint, mode)), 1e-9);
}

inline bool controlModeTargetNearActual(const CanDriverJointConfig &joint, AxisControlMode mode)
{
    const double target = controlModeSelectedCommandValue(joint, mode);
    const double actual = controlModeActualFeedbackValue(joint, mode);
    const double tolerance = controlModeAlignmentTolerance(joint, mode);
    return std::isfinite(target) && std::isfinite(actual) &&
           std::fabs(target - actual) <= tolerance;
}

inline void clearDirectCommandForControlMode(CanDriverJointConfig *joint, AxisControlMode mode)
{
    if (joint == nullptr) {
        return;
    }
    if (controlModeUsesVelocitySemantics(mode)) {
        joint->hasDirectVelCmd = false;
    } else {
        joint->hasDirectPosCmd = false;
    }
}

inline CanProtocol::MotorMode protocolMotorModeFromAxisControlMode(AxisControlMode mode)
{
    switch (mode) {
    case AxisControlMode::Velocity:
        return CanProtocol::MotorMode::Velocity;
    case AxisControlMode::Csp:
        return CanProtocol::MotorMode::CSP;
    case AxisControlMode::Position:
    case AxisControlMode::Unknown:
        return CanProtocol::MotorMode::Position;
    }
    return CanProtocol::MotorMode::Position;
}

} // namespace can_driver

#endif // CAN_DRIVER_AXIS_COMMAND_SEMANTICS_H
