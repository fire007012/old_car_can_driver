#include "can_driver/AxisCommandSemantics.h"
#include "can_driver/JointConfigParser.h"

#include <cmath>
#include <limits>
#include <stdexcept>

namespace joint_config_parser {

namespace {

constexpr double kDamiaoSiScale = 1e-4;

bool parsePositiveOptionalDouble(const XmlRpc::XmlRpcValue &jointValue,
                                 const char *fieldName,
                                 const std::string &jointName,
                                 double *out,
                                 std::string &errorMsg)
{
    if (!jointValue.hasMember(fieldName)) {
        return true;
    }

    const auto &value = jointValue[fieldName];
    if (value.getType() != XmlRpc::XmlRpcValue::TypeInt &&
        value.getType() != XmlRpc::XmlRpcValue::TypeDouble) {
        errorMsg = "Joint '" + jointName + "': invalid " + fieldName + ".";
        return false;
    }

    const double parsed = (value.getType() == XmlRpc::XmlRpcValue::TypeInt)
                              ? static_cast<double>(static_cast<int>(value))
                              : static_cast<double>(value);
    if (!std::isfinite(parsed) || parsed <= 0.0) {
        errorMsg = "Joint '" + jointName + "': invalid " + fieldName + ".";
        return false;
    }

    *out = parsed;
    return true;
}

bool parseNonNegativeOptionalDouble(const XmlRpc::XmlRpcValue &jointValue,
                                    const char *fieldName,
                                    const std::string &jointName,
                                    double *out,
                                    std::string &errorMsg)
{
    if (!jointValue.hasMember(fieldName)) {
        return true;
    }

    const auto &value = jointValue[fieldName];
    if (value.getType() != XmlRpc::XmlRpcValue::TypeInt &&
        value.getType() != XmlRpc::XmlRpcValue::TypeDouble) {
        errorMsg = "Joint '" + jointName + "': invalid " + fieldName + ".";
        return false;
    }

    const double parsed = (value.getType() == XmlRpc::XmlRpcValue::TypeInt)
                              ? static_cast<double>(static_cast<int>(value))
                              : static_cast<double>(value);
    if (!std::isfinite(parsed) || parsed < 0.0) {
        errorMsg = "Joint '" + jointName + "': invalid " + fieldName + ".";
        return false;
    }

    *out = parsed;
    return true;
}

bool parseFiniteOptionalDouble(const XmlRpc::XmlRpcValue &jointValue,
                               const char *fieldName,
                               const std::string &jointName,
                               double *out,
                               std::string &errorMsg)
{
    if (!jointValue.hasMember(fieldName)) {
        return true;
    }

    const auto &value = jointValue[fieldName];
    if (value.getType() != XmlRpc::XmlRpcValue::TypeInt &&
        value.getType() != XmlRpc::XmlRpcValue::TypeDouble) {
        errorMsg = "Joint '" + jointName + "': invalid " + fieldName + ".";
        return false;
    }

    const double parsed = (value.getType() == XmlRpc::XmlRpcValue::TypeInt)
                              ? static_cast<double>(static_cast<int>(value))
                              : static_cast<double>(value);
    if (!std::isfinite(parsed)) {
        errorMsg = "Joint '" + jointName + "': invalid " + fieldName + ".";
        return false;
    }

    *out = parsed;
    return true;
}

bool parseDirectionSign(const XmlRpc::XmlRpcValue &jointValue,
                        const std::string &jointName,
                        double *out,
                        std::string &errorMsg)
{
    if (!jointValue.hasMember("direction_sign")) {
        return true;
    }

    const auto &value = jointValue["direction_sign"];
    if (value.getType() != XmlRpc::XmlRpcValue::TypeInt &&
        value.getType() != XmlRpc::XmlRpcValue::TypeDouble) {
        errorMsg = "Joint '" + jointName + "': direction_sign must be either 1 or -1.";
        return false;
    }

    const double parsed = (value.getType() == XmlRpc::XmlRpcValue::TypeInt)
                              ? static_cast<double>(static_cast<int>(value))
                              : static_cast<double>(value);
    if (parsed != 1.0 && parsed != -1.0) {
        errorMsg = "Joint '" + jointName + "': direction_sign must be either 1 or -1.";
        return false;
    }

    *out = parsed;
    return true;
}

} // namespace

// 支持 int 或 string（十进制/十六进制）两种配置形式。
bool parseMotorId(const XmlRpc::XmlRpcValue &value,
                  const std::string &jointName,
                  MotorID &out,
                  std::string &errorMsg)
{
    int rawId = 0;
    if (value.getType() == XmlRpc::XmlRpcValue::TypeInt) {
        rawId = static_cast<int>(value);
    } else if (value.getType() == XmlRpc::XmlRpcValue::TypeString) {
        try {
            rawId = static_cast<int>(std::stoul(static_cast<std::string>(value), nullptr, 0));
        } catch (const std::exception &e) {
            errorMsg = "Joint '" + jointName + "': invalid motor_id '" +
                       static_cast<std::string>(value) + "' (" + e.what() + ").";
            return false;
        }
    } else {
        errorMsg = "Joint '" + jointName + "': motor_id must be int or string.";
        return false;
    }
    // MotorID 在系统中按 uint16_t 解释，解析时做边界保护。
    if (rawId < 0 || rawId > static_cast<int>(std::numeric_limits<uint16_t>::max())) {
        errorMsg = "Joint '" + jointName + "': motor_id out of range [0, 65535]: " +
                   std::to_string(rawId) + ".";
        return false;
    }
    out = static_cast<MotorID>(static_cast<uint16_t>(rawId));
    return true;
}

bool parse(const XmlRpc::XmlRpcValue &jointList,
           std::vector<ParsedJointConfig> &out,
           std::string &errorMsg)
{
    // joints 必须为非空数组，否则无法构建 hardware_interface 映射。
    if (jointList.getType() != XmlRpc::XmlRpcValue::TypeArray || jointList.size() == 0) {
        errorMsg = "'joints' must be a non-empty list.";
        return false;
    }

    out.clear();
    out.reserve(jointList.size());
    for (int i = 0; i < jointList.size(); ++i) {
        const auto &jv = jointList[i];
        // 必填字段保持严格校验，避免运行期出现“半配置”关节。
        if (!jv.hasMember("name") || !jv.hasMember("motor_id") ||
            !jv.hasMember("protocol") || !jv.hasMember("can_device") ||
            !jv.hasMember("control_mode")) {
            errorMsg = "Joint [" + std::to_string(i) +
                       "] missing required field (name/motor_id/protocol/can_device/control_mode).";
            return false;
        }

        ParsedJointConfig jc;
        jc.name = static_cast<std::string>(jv["name"]);
        jc.canDevice = static_cast<std::string>(jv["can_device"]);
        jc.controlMode = static_cast<std::string>(jv["control_mode"]);
        const auto axisMode = can_driver::axisControlModeFromString(jc.controlMode);
        if (axisMode == can_driver::AxisControlMode::Unknown) {
            errorMsg = "Joint '" + jc.name + "': unknown control_mode '" + jc.controlMode +
                       "' (use velocity, position, or csp).";
            return false;
        }

        // 协议字符串显式映射到枚举，未知值直接报错。
        const std::string protoStr = static_cast<std::string>(jv["protocol"]);
        if (protoStr == "MT") {
            jc.protocol = CanType::MT;
        } else if (protoStr == "PP") {
            jc.protocol = CanType::PP;
        } else if (protoStr == "DM") {
            jc.protocol = CanType::DM;
        } else if (protoStr == "ECB") {
            jc.protocol = CanType::ECB;
        } else {
            errorMsg = "Joint '" + jc.name +
                       "': unknown protocol '" + protoStr + "' (use MT/PP/DM/ECB).";
            return false;
        }

        if (!parseMotorId(jv["motor_id"], jc.name, jc.motorId, errorMsg)) {
            return false;
        }

        if (jc.protocol == CanType::DM &&
            can_driver::toProtocolNodeId(jc.motorId) > 0x0Fu) {
            errorMsg = "Joint '" + jc.name +
                       "': DM protocol currently requires motor_id low byte in [0, 15] "
                       "so feedback can be matched reliably.";
            return false;
        }

        if (jc.protocol == CanType::DM &&
            axisMode != can_driver::AxisControlMode::Velocity) {
            errorMsg = "Joint '" + jc.name +
                       "': DM protocol currently only supports control_mode 'velocity'.";
            return false;
        }

        // scale 支持两种填写方式：
        //   1. 直接填换算系数（小数，如 9.587e-05），raw * scale = rad
        //   2. 填每圈脉冲数 PPR（>=2 的整数，如 65536），自动换算为 2π/PPR
        if (jc.protocol == CanType::DM) {
            jc.positionScale = kDamiaoSiScale;
            jc.velocityScale = kDamiaoSiScale;
        }
        if (jv.hasMember("position_scale")) {
            const auto &sv = jv["position_scale"];
            const double val = (sv.getType() == XmlRpc::XmlRpcValue::TypeInt)
                                   ? static_cast<double>(static_cast<int>(sv))
                                   : static_cast<double>(sv);
            jc.positionScale = (val >= 2.0) ? (2.0 * M_PI / val) : val;
        }
        if (jv.hasMember("velocity_scale")) {
            const auto &sv = jv["velocity_scale"];
            const double val = (sv.getType() == XmlRpc::XmlRpcValue::TypeInt)
                                   ? static_cast<double>(static_cast<int>(sv))
                                   : static_cast<double>(sv);
            jc.velocityScale = (val >= 2.0) ? (2.0 * M_PI / val) : val;
        }
        if (!std::isfinite(jc.positionScale) || jc.positionScale <= 0.0) {
            errorMsg = "Joint '" + jc.name + "': invalid position_scale.";
            return false;
        }
        if (!std::isfinite(jc.velocityScale) || jc.velocityScale <= 0.0) {
            errorMsg = "Joint '" + jc.name + "': invalid velocity_scale.";
            return false;
        }
        if (!parseDirectionSign(jv, jc.name, &jc.directionSign, errorMsg)) {
            return false;
        }
        if (!parseFiniteOptionalDouble(jv, "zero_offset_rad", jc.name,
                                       &jc.zeroOffsetRad, errorMsg)) {
            return false;
        }
        if (!parsePositiveOptionalDouble(jv, "ip_max_velocity", jc.name,
                                         &jc.ipMaxVelocity, errorMsg)) {
            return false;
        }
        if (!parsePositiveOptionalDouble(jv, "ip_max_acceleration", jc.name,
                                         &jc.ipMaxAcceleration, errorMsg)) {
            return false;
        }
        if (!parsePositiveOptionalDouble(jv, "ip_max_jerk", jc.name,
                                         &jc.ipMaxJerk, errorMsg)) {
            return false;
        }
        if (!parseNonNegativeOptionalDouble(jv, "ip_goal_tolerance", jc.name,
                                            &jc.ipGoalTolerance, errorMsg)) {
            return false;
        }

        if (jv.hasMember("ecb_ip")) {
            if (jv["ecb_ip"].getType() != XmlRpc::XmlRpcValue::TypeString) {
                errorMsg = "Joint '" + jc.name + "': ecb_ip must be string.";
                return false;
            }
            jc.ecbIp = static_cast<std::string>(jv["ecb_ip"]);
        }
        if (jv.hasMember("ecb_discovery")) {
            if (jv["ecb_discovery"].getType() == XmlRpc::XmlRpcValue::TypeBoolean) {
                jc.ecbAutoDiscovery = static_cast<bool>(jv["ecb_discovery"]);
            } else if (jv["ecb_discovery"].getType() == XmlRpc::XmlRpcValue::TypeString) {
                const std::string mode = static_cast<std::string>(jv["ecb_discovery"]);
                if (mode == "auto") {
                    jc.ecbAutoDiscovery = true;
                } else if (mode == "fixed") {
                    jc.ecbAutoDiscovery = false;
                } else {
                    errorMsg = "Joint '" + jc.name + "': ecb_discovery must be auto/fixed/bool.";
                    return false;
                }
            } else {
                errorMsg = "Joint '" + jc.name + "': ecb_discovery must be auto/fixed/bool.";
                return false;
            }
        }
        if (jv.hasMember("ecb_refresh_ms")) {
            if (jv["ecb_refresh_ms"].getType() != XmlRpc::XmlRpcValue::TypeInt) {
                errorMsg = "Joint '" + jc.name + "': ecb_refresh_ms must be int.";
                return false;
            }
            jc.ecbRefreshMs = static_cast<int>(jv["ecb_refresh_ms"]);
            if (jc.ecbRefreshMs <= 0) {
                errorMsg = "Joint '" + jc.name + "': ecb_refresh_ms must be > 0.";
                return false;
            }
        }
        if (!parsePositiveOptionalDouble(jv, "ecb_profile_position_max_rpm", jc.name,
                                         &jc.ecbProfilePositionMaxRpm, errorMsg)) {
            return false;
        }
        if (!parsePositiveOptionalDouble(jv, "ecb_profile_position_acceleration_rpm_s", jc.name,
                                         &jc.ecbProfilePositionAccelerationRpmS, errorMsg)) {
            return false;
        }
        if (!parseFiniteOptionalDouble(jv, "ecb_profile_position_deceleration_rpm_s", jc.name,
                                       &jc.ecbProfilePositionDecelerationRpmS, errorMsg)) {
            return false;
        }
        if (!parsePositiveOptionalDouble(jv, "ecb_profile_velocity_acceleration_rpm_s", jc.name,
                                         &jc.ecbProfileVelocityAccelerationRpmS, errorMsg)) {
            return false;
        }
        if (!parseFiniteOptionalDouble(jv, "ecb_profile_velocity_deceleration_rpm_s", jc.name,
                                       &jc.ecbProfileVelocityDecelerationRpmS, errorMsg)) {
            return false;
        }

        if (jc.protocol == CanType::ECB && !jc.ecbAutoDiscovery && jc.ecbIp.empty()) {
            constexpr const char *kEcbPrefix = "ecb://";
            if (jc.canDevice.rfind(kEcbPrefix, 0) == 0) {
                const std::string payload = jc.canDevice.substr(6);
                if (!payload.empty() && payload != "auto") {
                    jc.ecbIp = payload;
                }
            }
        }

        out.push_back(jc);
    }

    return true;
}

} // namespace joint_config_parser
