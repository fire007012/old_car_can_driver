#ifndef CAN_DRIVER_MT_RMD_PROTOCOL_H
#define CAN_DRIVER_MT_RMD_PROTOCOL_H

#include "can_driver/CanTransport.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cstddef>

namespace can_driver::mt_rmd {

constexpr std::uint16_t kSendBaseId = 0x140;
constexpr std::uint16_t kResponseBaseId = 0x240;

enum class Command : std::uint8_t {
    WriteAccelerationToRamAndRom = 0x43,
    WriteCurrentMultiTurnPositionToRomAsZero = 0x64,
    ResetSystem = 0x76,
    ShutdownMotor = 0x80,
    StopMotor = 0x81,
    ReadMultiTurnAngle = 0x92,
    ReadMotorStatus1AndErrorFlag = 0x9A,
    ReadMotorStatus2 = 0x9C,
    TorqueClosedLoopControl = 0xA1,
    SpeedClosedLoopControl = 0xA2,
    AbsolutePositionClosedLoopControl = 0xA4,
    SingleTurnPositionControl = 0xA6,
    IncrementalPositionClosedLoopControl = 0xA8,
    ForcePositionMixedControl = 0xA9,
    CommunicationInterruptionProtectionTimeSetting = 0xB3,
};

enum class AccelerationType : std::uint8_t {
    PositionPlanningAcceleration = 0x00,
    PositionPlanningDeceleration = 0x01,
    VelocityPlanningAcceleration = 0x02,
    VelocityPlanningDeceleration = 0x03,
};

struct FeedbackStatus {
    std::int8_t temperature = 0;
    double currentAmp = 0.0;
    std::int16_t shaftSpeedDps = 0;
    std::uint16_t shaftAngleRaw = 0;
};

struct FaultStatus {
    std::int8_t temperature = 0;
    std::uint16_t voltageRaw = 0;
    std::uint16_t brakeOrVoltageRaw = 0;
    std::uint16_t errorCode = 0;
};

constexpr std::uint8_t commandByte(Command command)
{
    return static_cast<std::uint8_t>(command);
}

constexpr std::uint16_t sendCanId(std::uint8_t nodeId)
{
    return static_cast<std::uint16_t>(kSendBaseId + nodeId);
}

constexpr bool isResponseCanId(std::uint16_t canId)
{
    return canId >= kResponseBaseId && canId < static_cast<std::uint16_t>(kResponseBaseId + 0x100);
}

constexpr std::uint8_t responseNodeId(std::uint16_t canId)
{
    return static_cast<std::uint8_t>(canId - kResponseBaseId);
}

inline CanTransport::Frame makeFrame(std::uint16_t canId, Command command)
{
    CanTransport::Frame frame;
    frame.id = canId;
    frame.dlc = 8;
    frame.isExtended = false;
    frame.isRemoteRequest = false;
    frame.data.fill(0);
    frame.data[0] = commandByte(command);
    return frame;
}

inline void writeUInt16LE(CanTransport::Frame &frame, std::size_t index, std::uint16_t value)
{
    frame.data[index] = static_cast<std::uint8_t>(value & 0xFF);
    frame.data[index + 1] = static_cast<std::uint8_t>((value >> 8) & 0xFF);
}

inline void writeInt32LE(CanTransport::Frame &frame, std::size_t index, std::int32_t value)
{
    const auto bits = static_cast<std::uint32_t>(value);
    frame.data[index] = static_cast<std::uint8_t>(bits & 0xFF);
    frame.data[index + 1] = static_cast<std::uint8_t>((bits >> 8) & 0xFF);
    frame.data[index + 2] = static_cast<std::uint8_t>((bits >> 16) & 0xFF);
    frame.data[index + 3] = static_cast<std::uint8_t>((bits >> 24) & 0xFF);
}

inline void writeUInt32LE(CanTransport::Frame &frame, std::size_t index, std::uint32_t value)
{
    frame.data[index] = static_cast<std::uint8_t>(value & 0xFF);
    frame.data[index + 1] = static_cast<std::uint8_t>((value >> 8) & 0xFF);
    frame.data[index + 2] = static_cast<std::uint8_t>((value >> 16) & 0xFF);
    frame.data[index + 3] = static_cast<std::uint8_t>((value >> 24) & 0xFF);
}

inline std::int16_t readInt16LE(const CanTransport::Frame &frame, std::size_t index)
{
    if (index + 1 >= frame.dlc) {
        return 0;
    }
    const int value = static_cast<int>(frame.data[index]) |
                      (static_cast<int>(frame.data[index + 1]) << 8);
    return static_cast<std::int16_t>(value);
}

inline std::uint16_t readUInt16LE(const CanTransport::Frame &frame, std::size_t index)
{
    if (index + 1 >= frame.dlc) {
        return 0;
    }
    return static_cast<std::uint16_t>(static_cast<std::uint16_t>(frame.data[index]) |
                                      (static_cast<std::uint16_t>(frame.data[index + 1]) << 8));
}

inline std::int32_t readInt32LE(const CanTransport::Frame &frame, std::size_t index)
{
    if (index + 3 >= frame.dlc) {
        return 0;
    }
    const std::uint32_t value = static_cast<std::uint32_t>(frame.data[index]) |
                                (static_cast<std::uint32_t>(frame.data[index + 1]) << 8) |
                                (static_cast<std::uint32_t>(frame.data[index + 2]) << 16) |
                                (static_cast<std::uint32_t>(frame.data[index + 3]) << 24);
    return static_cast<std::int32_t>(value);
}

inline std::uint32_t clampAcceleration(std::uint32_t value)
{
    return std::max<std::uint32_t>(100, std::min<std::uint32_t>(60000, value));
}

inline CanTransport::Frame makeSpeedClosedLoopFrame(std::uint16_t canId,
                                                    std::int32_t speedCentidegPerSec)
{
    auto frame = makeFrame(canId, Command::SpeedClosedLoopControl);
    writeInt32LE(frame, 4, speedCentidegPerSec);
    return frame;
}

inline CanTransport::Frame makeAbsolutePositionFrame(std::uint16_t canId,
                                                     std::int32_t positionCentideg,
                                                     std::uint16_t maxSpeedDps)
{
    auto frame = makeFrame(canId, Command::AbsolutePositionClosedLoopControl);
    writeUInt16LE(frame, 2, maxSpeedDps);
    writeInt32LE(frame, 4, positionCentideg);
    return frame;
}

inline CanTransport::Frame makeAccelerationFrame(std::uint16_t canId,
                                                 AccelerationType type,
                                                 std::uint32_t accelerationDpsPerSec)
{
    auto frame = makeFrame(canId, Command::WriteAccelerationToRamAndRom);
    frame.data[1] = static_cast<std::uint8_t>(type);
    writeUInt32LE(frame, 4, clampAcceleration(accelerationDpsPerSec));
    return frame;
}

inline CanTransport::Frame makeCommunicationTimeoutFrame(std::uint16_t canId,
                                                         std::chrono::milliseconds timeout)
{
    auto frame = makeFrame(canId, Command::CommunicationInterruptionProtectionTimeSetting);
    writeUInt32LE(frame, 4, static_cast<std::uint32_t>(timeout.count()));
    return frame;
}

inline CanTransport::Frame makeReadFrame(std::uint16_t canId, Command command)
{
    return makeFrame(canId, command);
}

inline CanTransport::Frame makeZeroPositionFrame(std::uint16_t canId)
{
    return makeFrame(canId, Command::WriteCurrentMultiTurnPositionToRomAsZero);
}

inline CanTransport::Frame makeResetFrame(std::uint16_t canId)
{
    return makeFrame(canId, Command::ResetSystem);
}

inline CanTransport::Frame makeShutdownFrame(std::uint16_t canId)
{
    return makeFrame(canId, Command::ShutdownMotor);
}

inline CanTransport::Frame makeStopFrame(std::uint16_t canId)
{
    return makeFrame(canId, Command::StopMotor);
}

inline FeedbackStatus parseFeedbackStatus(const CanTransport::Frame &frame)
{
    FeedbackStatus status;
    if (frame.dlc >= 8) {
        status.temperature = static_cast<std::int8_t>(frame.data[1]);
        status.currentAmp = static_cast<double>(readInt16LE(frame, 2)) * 0.01;
        status.shaftSpeedDps = readInt16LE(frame, 4);
        status.shaftAngleRaw = readUInt16LE(frame, 6);
    }
    return status;
}

inline FaultStatus parseFaultStatus(const CanTransport::Frame &frame)
{
    FaultStatus status;
    if (frame.dlc >= 8) {
        status.temperature = static_cast<std::int8_t>(frame.data[1]);
        status.voltageRaw = readUInt16LE(frame, 2);
        status.brakeOrVoltageRaw = readUInt16LE(frame, 4);
        status.errorCode = readUInt16LE(frame, 6);
    }
    return status;
}

inline std::int64_t parseMultiTurnAngleCentideg(const CanTransport::Frame &frame)
{
    return readInt32LE(frame, 4);
}

inline std::uint32_t parseCommunicationTimeoutMs(const CanTransport::Frame &frame)
{
    if (frame.dlc < 8) {
        return 0;
    }
    return static_cast<std::uint32_t>(frame.data[4]) |
           (static_cast<std::uint32_t>(frame.data[5]) << 8) |
           (static_cast<std::uint32_t>(frame.data[6]) << 16) |
           (static_cast<std::uint32_t>(frame.data[7]) << 24);
}

inline std::uint16_t positionMaxSpeedDpsFromSpeedCommand(std::int32_t speedCentidegPerSec,
                                                         std::uint16_t defaultSpeedDps)
{
    const auto absSpeedCentidegPerSec = std::llabs(static_cast<long long>(speedCentidegPerSec));
    if (absSpeedCentidegPerSec == 0) {
        return defaultSpeedDps;
    }

    auto speedDps = (absSpeedCentidegPerSec + 50) / 100;
    if (speedDps == 0) {
        speedDps = 1;
    }
    return static_cast<std::uint16_t>(std::min<long long>(speedDps, 0xFFFF));
}

inline bool isFeedbackCommand(std::uint8_t command)
{
    return command == commandByte(Command::ReadMotorStatus2) ||
           command == commandByte(Command::TorqueClosedLoopControl) ||
           command == commandByte(Command::SpeedClosedLoopControl) ||
           command == commandByte(Command::AbsolutePositionClosedLoopControl) ||
           command == commandByte(Command::SingleTurnPositionControl) ||
           command == commandByte(Command::IncrementalPositionClosedLoopControl) ||
           command == commandByte(Command::ForcePositionMixedControl) ||
           command == commandByte(Command::ShutdownMotor) ||
           command == commandByte(Command::StopMotor);
}

} // namespace can_driver::mt_rmd

#endif // CAN_DRIVER_MT_RMD_PROTOCOL_H