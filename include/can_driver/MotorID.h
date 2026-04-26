#ifndef MOTORID_H
#define MOTORID_H

#include <cstdint>

/**
 * @brief 设备/电机的逻辑 ID。
 *
 * 值使用完整的 16 位 CAN ID，方便与现有宏定义保持一致；
 * 通过辅助函数可以取得低 8 位索引用于调用 CanProtocol 接口。
 */
enum class MotorID : std::uint16_t {
    LeftWheel = 0x141,
    RightWheel = 0x142,
    
    RotaryTable = 0x6,
    LargerArm = 0x5,
    SmallerArm = 0x13,
    WristX = 0x14,
    WristY = 0x15,
    WristZ = 0x16,
    Actuator = 0x1E,

    PTZ1 = 0x21,
    PTZ2 = 0x22,

    SwingArmLeftForward = 0x1,
    SwingArmRightForward = 0x2,
    SwingArmLeftBackward = 0x3,
    SwingArmRightBackward = 0x4,
};

// enum class EyouMotorID : std::uint16_t {

//     Arm3 = 0x13,
//     Arm4 = 0x14,
//     Arm5 = 0x15,
//     Arm6 = 0x16,
//     TailEnd = 0x1E,

// };

// enum class CanOpenMotorID : std::uint16_t {

//     Arm1 = 6,
//     Arm2 = 5,

//     SwingArm1 = 1,
//     SwingArm2 = 2,
//     SwingArm3 = 3,
//     SwingArm4 = 4

// };

// enum class MTMotorID : std::uint16_t {

//     MainWheel1 = 0x141,
//     MainWheel2 = 0x142,

//     PTZ1 = 0x21,
//     PTZ2 = 0x22,

// };
// /**
//  * @brief 返回 电机ID 对应的完整 CAN ID。
//  */
// constexpr std::uint16_t toCanId(EyouMotorID id)
// {
//     return static_cast<std::uint16_t>(id);
// }
// constexpr std::uint16_t toCanId(CanOpenMotorID id)
// {
//     return static_cast<std::uint16_t>(id);
// }
// constexpr std::uint16_t toCanId(MTMotorID id)
// {
//     return static_cast<std::uint16_t>(id);
// }

/**
 * @brief 返回 电机ID 对应的完整 CAN ID。
 */
constexpr std::uint16_t toCanId(MotorID id)
{
    return static_cast<std::uint16_t>(id);
}

namespace can_driver {

constexpr std::uint16_t kMtSendBaseId = 0x140u;

/**
 * @brief 返回系统层 motor_id（对外服务、状态发布、配置解析使用的稳定 ID）。
 */
constexpr std::uint16_t toSystemMotorId(MotorID id)
{
    return static_cast<std::uint16_t>(id);
}

/**
 * @brief 将系统层 motor_id 转换为 MT 协议在线路上使用的 node id。
 *
 * MT 在系统配置中通常写完整发送 CAN ID，例如 0x141~0x15F；
 * 实际协议节点号需要减去发送基址 0x140。
 */
constexpr std::uint8_t toMtProtocolNodeId(MotorID id)
{
    const auto systemId = toSystemMotorId(id);
    return static_cast<std::uint8_t>(systemId >= kMtSendBaseId ? (systemId - kMtSendBaseId)
                                                               : (systemId & 0xFFu));
}

/**
 * @brief 根据 MT 协议 node id 还原系统层 motor_id。
 */
constexpr MotorID motorIdFromMtProtocolNodeId(std::uint8_t nodeId)
{
    return static_cast<MotorID>(static_cast<std::uint16_t>(kMtSendBaseId + nodeId));
}

/**
 * @brief 返回协议在线上使用的 node id。
 *
 * 目前 MT/PP 协议都以低 8 位作为总线节点号；
 * 系统层仍保留完整 16 位 motor_id，以避免上层寻址语义被协议细节污染。
 */
constexpr std::uint8_t toProtocolNodeId(MotorID id)
{
    return static_cast<std::uint8_t>(toSystemMotorId(id) & 0xFFu);
}

/**
 * @brief 当仅知道协议 node id 时，构造一个最保守的系统 motor_id 回退值。
 *
 * 该回退值只用于协议单测/未完成注册映射时的兜底；
 * 真正运行中应优先使用配置阶段建立的“系统 id -> node id”映射。
 */
constexpr MotorID motorIdFromProtocolNodeId(std::uint8_t nodeId)
{
    return static_cast<MotorID>(static_cast<std::uint16_t>(nodeId));
}

} // namespace can_driver

#endif // MOTORID_H
