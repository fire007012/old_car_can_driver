#include <gtest/gtest.h>
#include <ros/master.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

#include "can_driver/ApplyLimits.h"
#include "can_driver/CanDriverHW.h"
#include "can_driver/CanDriverIoRuntime.h"
#include "can_driver/IDeviceManager.h"
#include "can_driver/MotorState.h"
#include "can_driver/lifecycle_service_gateway.hpp"
#include "can_driver/MotorCommand.h"
#include "can_driver/Recover.h"
#include "can_driver/SetZero.h"
#include "can_driver/SetZeroLimit.h"

#include <atomic>
#include <cmath>
#include <deque>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace {

class FakeProtocol : public CanProtocol {
public:
    bool setMode(MotorID motorId, MotorMode mode) override
    {
        std::lock_guard<std::mutex> lock(mutex_);
        lastModeMotor_ = motorId;
        lastMode_ = mode;
        ++setModeCalls_;
        return setModeResult_;
    }

    bool setVelocity(MotorID motorId, int32_t velocity) override
    {
        std::lock_guard<std::mutex> lock(mutex_);
        lastVelocityMotor_ = motorId;
        lastVelocity_ = velocity;
        ++setVelocityCalls_;
        return true;
    }

    bool setAcceleration(MotorID, int32_t) override { return true; }
    bool setDeceleration(MotorID, int32_t) override { return true; }

    bool setPosition(MotorID motorId, int32_t position) override
    {
        std::lock_guard<std::mutex> lock(mutex_);
        lastPositionMotor_ = motorId;
        lastPosition_ = position;
        ++setPositionCalls_;
        return true;
    }

    bool quickSetPosition(MotorID motorId, int32_t position) override
    {
        std::lock_guard<std::mutex> lock(mutex_);
        lastQuickPositionMotor_ = motorId;
        lastQuickPosition_ = position;
        ++quickSetPositionCalls_;
        return true;
    }

    bool Enable(MotorID motorId) override
    {
        std::lock_guard<std::mutex> lock(mutex_);
        lastEnableMotor_ = motorId;
        enabled_ = true;
        ++enableCalls_;
        return true;
    }

    bool Disable(MotorID) override
    {
        std::lock_guard<std::mutex> lock(mutex_);
        enabled_ = false;
        return true;
    }

    bool Stop(MotorID) override
    {
        std::lock_guard<std::mutex> lock(mutex_);
        ++stopCalls_;
        return true;
    }

    bool ResetFault(MotorID) override
    {
        std::lock_guard<std::mutex> lock(mutex_);
        hasFault_ = false;
        ++resetFaultCalls_;
        return true;
    }

    bool configurePositionLimits(MotorID motorId,
                                 int32_t minPositionRaw,
                                 int32_t maxPositionRaw,
                                 bool enable) override
    {
        std::lock_guard<std::mutex> lock(mutex_);
        lastLimitMotor_ = motorId;
        lastLimitMin_ = minPositionRaw;
        lastLimitMax_ = maxPositionRaw;
        lastLimitEnable_ = enable;
        ++setLimitCalls_;
        return true;
    }

    bool setPositionOffset(MotorID motorId, int32_t offsetRaw) override
    {
        std::lock_guard<std::mutex> lock(mutex_);
        lastOffsetMotor_ = motorId;
        lastOffsetRaw_ = offsetRaw;
        ++setOffsetCalls_;
        return setOffsetResult_;
    }

    bool readPositionOffset(MotorID, int32_t* offsetRaw) override
    {
        if (offsetRaw == nullptr) {
            return false;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        *offsetRaw = lastOffsetRaw_;
        return true;
    }

    bool readSerialNumber(MotorID, uint32_t* serialNumber) override
    {
        if (serialNumber == nullptr) {
            return false;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        if (!serialNumberReadable_) {
            return false;
        }
        *serialNumber = serialNumber_;
        return true;
    }

    bool persistParameters(MotorID motorId) override
    {
        std::lock_guard<std::mutex> lock(mutex_);
        lastPersistMotor_ = motorId;
        ++persistCalls_;
        return persistResult_;
    }

    int64_t getPosition(MotorID) const override
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return feedbackPosition_;
    }

    int16_t getCurrent(MotorID) const override
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return feedbackCurrent_;
    }

    int32_t getVelocity(MotorID) const override
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return feedbackVelocity_;
    }

    void initializeMotorRefresh(const std::vector<MotorID> &) override {}

    int setModeCalls() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return setModeCalls_;
    }

    uint16_t lastModeMotor() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return static_cast<uint16_t>(lastModeMotor_);
    }

    MotorMode lastMode() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return lastMode_;
    }

    int velocityCalls() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return setVelocityCalls_;
    }

    int32_t lastVelocity() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return lastVelocity_;
    }

    uint16_t lastVelocityMotor() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return static_cast<uint16_t>(lastVelocityMotor_);
    }

    int enableCalls() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return enableCalls_;
    }

    int positionCalls() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return setPositionCalls_;
    }

    int32_t lastPosition() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return lastPosition_;
    }

    uint16_t lastPositionMotor() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return static_cast<uint16_t>(lastPositionMotor_);
    }

    int quickPositionCalls() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return quickSetPositionCalls_;
    }

    int32_t lastQuickPosition() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return lastQuickPosition_;
    }

    uint16_t lastQuickPositionMotor() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return static_cast<uint16_t>(lastQuickPositionMotor_);
    }

    uint16_t lastEnableMotor() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return static_cast<uint16_t>(lastEnableMotor_);
    }

    [[nodiscard]] bool isEnabled(MotorID) const override
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return enabled_;
    }

    [[nodiscard]] bool hasFault(MotorID) const override
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return hasFault_;
    }

    void setFault(bool value)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        hasFault_ = value;
    }

    void setModeResult(bool value)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        setModeResult_ = value;
    }

    void setEnabledState(bool value)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        enabled_ = value;
    }

    void setFeedbackPosition(int64_t value)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        feedbackPosition_ = value;
    }

    void setFeedbackVelocity(int16_t value)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        feedbackVelocity_ = value;
    }

    void setFeedbackCurrent(int16_t value)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        feedbackCurrent_ = value;
    }

    int stopCalls() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return stopCalls_;
    }

    int resetFaultCalls() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return resetFaultCalls_;
    }

    int setOffsetCalls() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return setOffsetCalls_;
    }

    int persistCalls() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return persistCalls_;
    }

    int setLimitCalls() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return setLimitCalls_;
    }

    int32_t lastOffsetRaw() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return lastOffsetRaw_;
    }

    uint16_t lastPersistMotor() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return static_cast<uint16_t>(lastPersistMotor_);
    }

    int32_t lastLimitMin() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return lastLimitMin_;
    }

    int32_t lastLimitMax() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return lastLimitMax_;
    }

    bool lastLimitEnable() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return lastLimitEnable_;
    }

    void setOffsetResult(bool value)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        setOffsetResult_ = value;
    }

    void setPersistResult(bool value)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        persistResult_ = value;
    }

    void setSerialNumberReadable(bool value)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        serialNumberReadable_ = value;
    }

    void setSerialNumber(uint32_t value)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        serialNumber_ = value;
    }

private:
    mutable std::mutex mutex_;
    MotorID lastModeMotor_{MotorID::LeftWheel};
    MotorMode lastMode_{MotorMode::Position};
    int setModeCalls_{0};
    bool setModeResult_{true};

    MotorID lastVelocityMotor_{MotorID::LeftWheel};
    int32_t lastVelocity_{0};
    int setVelocityCalls_{0};

    MotorID lastPositionMotor_{MotorID::LeftWheel};
    int32_t lastPosition_{0};
    int setPositionCalls_{0};

    MotorID lastQuickPositionMotor_{MotorID::LeftWheel};
    int32_t lastQuickPosition_{0};
    int quickSetPositionCalls_{0};

    MotorID lastEnableMotor_{MotorID::LeftWheel};
    int enableCalls_{0};
    int stopCalls_{0};
    int resetFaultCalls_{0};
    MotorID lastOffsetMotor_{MotorID::LeftWheel};
    int32_t lastOffsetRaw_{0};
    int setOffsetCalls_{0};
    bool setOffsetResult_{true};
    MotorID lastPersistMotor_{MotorID::LeftWheel};
    int persistCalls_{0};
    bool persistResult_{true};
    bool serialNumberReadable_{true};
    uint32_t serialNumber_{0x12345678u};
    MotorID lastLimitMotor_{MotorID::LeftWheel};
    int32_t lastLimitMin_{0};
    int32_t lastLimitMax_{0};
    bool lastLimitEnable_{false};
    int setLimitCalls_{0};
    bool enabled_{true};
    bool hasFault_{false};
    int64_t feedbackPosition_{0};
    int16_t feedbackVelocity_{0};
    int16_t feedbackCurrent_{0};
};

class FakeDeviceManager : public IDeviceManager {
public:
    explicit FakeDeviceManager(bool exposeSharedState = true)
        : protocol_(std::make_shared<FakeProtocol>())
        , mutex_(std::make_shared<std::mutex>())
        , sharedState_(std::make_shared<can_driver::SharedDriverState>())
        , exposeSharedState_(exposeSharedState)
    {
    }

    bool ensureTransport(const std::string &device, bool = false) override
    {
        std::lock_guard<std::mutex> lock(lifecycleMutex_);
        ++ensureTransportCalls_;
        ensuredDevices_.insert(device);
        return true;
    }
    bool ensureProtocol(const std::string &device, CanType type) override
    {
        std::lock_guard<std::mutex> lock(lifecycleMutex_);
        ++ensureProtocolCalls_;
        ensuredProtocols_.insert({device, type});
        return true;
    }

    bool initDevice(const std::string &device,
                    const std::vector<std::pair<CanType, MotorID>> &motors,
                    bool = false) override
    {
        std::lock_guard<std::mutex> lock(lifecycleMutex_);
        ++initDeviceCalls_;
        initializedDevices_.insert(device);
        initializedMotors_[device] = motors;
        sharedState_->mutateDeviceHealth(
            device,
            [](can_driver::SharedDriverState::DeviceHealthState *health) {
                health->transportReady = true;
            });
        if (seedFeedbackOnInit_) {
            const auto nowNs = can_driver::SharedDriverSteadyNowNs();
            const bool velocityOnlyFeedback = velocityOnlyFeedbackOnInit_;
            for (const auto &entry : motors) {
                const auto key =
                    can_driver::MakeAxisKey(device, entry.first, entry.second);
                sharedState_->registerAxis(key);
                sharedState_->mutateAxisFeedback(
                    key,
                    [this, nowNs, &entry, velocityOnlyFeedback](
                        can_driver::SharedDriverState::AxisFeedbackState *feedback) {
                        feedback->feedbackSeen = true;
                        feedback->positionValid = !velocityOnlyFeedback;
                        feedback->velocityValid = true;
                        feedback->currentValid = !velocityOnlyFeedback;
                        feedback->modeValid = true;
                        feedback->position = protocol_->getPosition(entry.second);
                        feedback->velocity = protocol_->getVelocity(entry.second);
                        feedback->current = protocol_->getCurrent(entry.second);
                        feedback->mode = protocol_->lastMode();
                        feedback->enabled = protocol_->isEnabled(entry.second);
                        feedback->fault = protocol_->hasFault(entry.second);
                        feedback->enabledValid = true;
                        feedback->faultValid = true;
                        feedback->lastRxSteadyNs = nowNs;
                        feedback->lastValidStateSteadyNs = nowNs;
                    });
            }
        }
        return true;
    }

    void startRefresh(const std::string &, CanType, const std::vector<MotorID> &) override {}
    void setRefreshRateHz(double hz) override
    {
        std::lock_guard<std::mutex> lock(lifecycleMutex_);
        refreshRateHzCalls_.push_back(hz);
    }
    void setDeviceRefreshRateHz(const std::string &device, double hz) override
    {
        std::lock_guard<std::mutex> lock(lifecycleMutex_);
        deviceRefreshRateHzCalls_.emplace_back(device, hz);
    }
    void setRefreshInterFrameGapUs(uint32_t) override {}
    void setMtCommunicationTimeoutOnInitMs(uint32_t timeoutMs) override
    {
        std::lock_guard<std::mutex> lock(lifecycleMutex_);
        mtCommunicationTimeoutOnInitMsCalls_.push_back(timeoutMs);
    }
    void setPpFastWriteEnabled(bool) override {}
    void setPpDefaultPositionVelocityRaw(int32_t) override {}
    void setPpPositionDefaultVelocityRaw(int32_t) override {}
    void setPpCspDefaultVelocityRaw(int32_t) override {}
    void shutdownDevice(const std::string &device) override
    {
        std::lock_guard<std::mutex> lock(lifecycleMutex_);
        ++shutdownDeviceCalls_;
        lastShutdownDevice_ = device;
        initializedDevices_.erase(device);
        initializedMotors_.erase(device);
        sharedState_->mutateDeviceHealth(
            device,
            [](can_driver::SharedDriverState::DeviceHealthState *health) {
                health->transportReady = false;
            });
    }
    void shutdownAll() override
    {
        std::lock_guard<std::mutex> lock(lifecycleMutex_);
        initializedDevices_.clear();
    }

    std::shared_ptr<CanProtocol> getProtocol(const std::string &, CanType) const override
    {
        return protocol_;
    }

    std::shared_ptr<std::mutex> getDeviceMutex(const std::string &) const override
    {
        return mutex_;
    }

    bool isDeviceReady(const std::string &) const override
    {
        std::lock_guard<std::mutex> lock(readyMutex_);
        if (!ready_ || !exposeSharedState_ || !seedFeedbackOnInit_ ||
            !syncSharedFeedbackFromProtocol_) {
            return ready_;
        }

        const auto nowNs = can_driver::SharedDriverSteadyNowNs();
        std::lock_guard<std::mutex> lifecycleLock(lifecycleMutex_);
        const bool velocityOnlyFeedback = velocityOnlyFeedbackOnInit_;
        for (const auto &deviceEntry : initializedMotors_) {
            sharedState_->mutateDeviceHealth(
                deviceEntry.first,
                [](can_driver::SharedDriverState::DeviceHealthState *health) {
                    health->transportReady = true;
                });
            for (const auto &motorEntry : deviceEntry.second) {
                const auto key = can_driver::MakeAxisKey(
                    deviceEntry.first, motorEntry.first, motorEntry.second);
                sharedState_->mutateAxisFeedback(
                    key,
                    [this, nowNs, &motorEntry, velocityOnlyFeedback](
                        can_driver::SharedDriverState::AxisFeedbackState *feedback) {
                        feedback->feedbackSeen = true;
                        feedback->positionValid = !velocityOnlyFeedback;
                        feedback->velocityValid = true;
                        feedback->currentValid = !velocityOnlyFeedback;
                        feedback->modeValid = true;
                        feedback->position = protocol_->getPosition(motorEntry.second);
                        feedback->velocity = protocol_->getVelocity(motorEntry.second);
                        feedback->current = protocol_->getCurrent(motorEntry.second);
                        feedback->mode = protocol_->lastMode();
                        feedback->enabled = protocol_->isEnabled(motorEntry.second);
                        feedback->fault = protocol_->hasFault(motorEntry.second);
                        feedback->enabledValid = true;
                        feedback->faultValid = true;
                        feedback->lastRxSteadyNs = nowNs;
                        feedback->lastValidStateSteadyNs = nowNs;
                    });
            }
        }
        return true;
    }
    std::size_t deviceCount() const override
    {
        std::lock_guard<std::mutex> lock(lifecycleMutex_);
        return initializedDevices_.size();
    }
    std::shared_ptr<can_driver::SharedDriverState> getSharedDriverState() const override
    {
        return exposeSharedState_ ? sharedState_ : nullptr;
    }

    std::shared_ptr<FakeProtocol> protocol() const { return protocol_; }
    std::shared_ptr<can_driver::SharedDriverState> sharedState() const { return sharedState_; }
    void setReady(bool ready)
    {
        std::lock_guard<std::mutex> lock(readyMutex_);
        ready_ = ready;
    }

    int shutdownDeviceCalls() const
    {
        std::lock_guard<std::mutex> lock(lifecycleMutex_);
        return shutdownDeviceCalls_;
    }

    std::string lastShutdownDevice() const
    {
        std::lock_guard<std::mutex> lock(lifecycleMutex_);
        return lastShutdownDevice_;
    }

    int ensureTransportCalls() const
    {
        std::lock_guard<std::mutex> lock(lifecycleMutex_);
        return ensureTransportCalls_;
    }

    int ensureProtocolCalls() const
    {
        std::lock_guard<std::mutex> lock(lifecycleMutex_);
        return ensureProtocolCalls_;
    }

    int initDeviceCalls() const
    {
        std::lock_guard<std::mutex> lock(lifecycleMutex_);
        return initDeviceCalls_;
    }

    void setSeedFeedbackOnInit(bool seed)
    {
        std::lock_guard<std::mutex> lock(lifecycleMutex_);
        seedFeedbackOnInit_ = seed;
    }

    void setVelocityOnlyFeedbackOnInit(bool velocityOnly)
    {
        std::lock_guard<std::mutex> lock(lifecycleMutex_);
        velocityOnlyFeedbackOnInit_ = velocityOnly;
    }

    void setSyncSharedFeedbackFromProtocol(bool sync)
    {
        std::lock_guard<std::mutex> lock(readyMutex_);
        syncSharedFeedbackFromProtocol_ = sync;
    }

    std::vector<double> refreshRateHzCalls() const
    {
        std::lock_guard<std::mutex> lock(lifecycleMutex_);
        return refreshRateHzCalls_;
    }

    std::vector<std::pair<std::string, double>> deviceRefreshRateHzCalls() const
    {
        std::lock_guard<std::mutex> lock(lifecycleMutex_);
        return deviceRefreshRateHzCalls_;
    }

    std::vector<uint32_t> mtCommunicationTimeoutOnInitMsCalls() const
    {
        std::lock_guard<std::mutex> lock(lifecycleMutex_);
        return mtCommunicationTimeoutOnInitMsCalls_;
    }

private:
    std::shared_ptr<FakeProtocol> protocol_;
    std::shared_ptr<std::mutex> mutex_;
    std::shared_ptr<can_driver::SharedDriverState> sharedState_;
    bool exposeSharedState_{false};
    mutable std::mutex readyMutex_;
    mutable std::mutex lifecycleMutex_;
    bool ready_{true};
    bool syncSharedFeedbackFromProtocol_{true};
    int shutdownDeviceCalls_{0};
    std::string lastShutdownDevice_;
    int ensureTransportCalls_{0};
    int ensureProtocolCalls_{0};
    int initDeviceCalls_{0};
    bool seedFeedbackOnInit_{true};
    bool velocityOnlyFeedbackOnInit_{false};
    std::vector<double> refreshRateHzCalls_;
    std::vector<std::pair<std::string, double>> deviceRefreshRateHzCalls_;
    std::vector<uint32_t> mtCommunicationTimeoutOnInitMsCalls_;
    std::set<std::string> ensuredDevices_;
    std::set<std::pair<std::string, CanType>> ensuredProtocols_;
    std::set<std::string> initializedDevices_;
    std::map<std::string, std::vector<std::pair<CanType, MotorID>>> initializedMotors_;
};

class CanDriverHWSmokeTest : public ::testing::Test {
protected:
    static void SetUpTestSuite()
    {
        if (!ros::isInitialized()) {
            int argc = 0;
            char **argv = nullptr;
            ros::init(argc, argv, "test_can_driver_hw_smoke",
                      ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);
        }
        ros::Time::init();
        ASSERT_TRUE(ros::master::check())
            << "test_can_driver_hw_smoke requires a running ROS master; run it via rostest.";
    }

    static XmlRpc::XmlRpcValue makeSingleVelocityJoint(double directionSign = 1.0)
    {
        XmlRpc::XmlRpcValue joints;
        joints.setSize(1);
        joints[0]["name"] = "test_wheel";
        joints[0]["motor_id"] = static_cast<int>(0x141);
        joints[0]["protocol"] = "MT";
        joints[0]["can_device"] = "fake0";
        joints[0]["control_mode"] = "velocity";
        joints[0]["velocity_scale"] = 0.1;
        joints[0]["position_scale"] = 1.0;
        joints[0]["direction_sign"] = directionSign;
        return joints;
    }

    static XmlRpc::XmlRpcValue makeSingleCspJoint(double directionSign = 1.0)
    {
        XmlRpc::XmlRpcValue joints;
        joints.setSize(1);
        joints[0]["name"] = "test_arm";
        joints[0]["motor_id"] = 0x05;
        joints[0]["protocol"] = "PP";
        joints[0]["can_device"] = "fake0";
        joints[0]["control_mode"] = "csp";
        joints[0]["position_scale"] = 65536;
        joints[0]["velocity_scale"] = 65536;
        joints[0]["direction_sign"] = directionSign;
        return joints;
    }

    static XmlRpc::XmlRpcValue makeSingleMtPositionJoint(double zeroOffsetRad = 0.0)
    {
        XmlRpc::XmlRpcValue joints;
        joints.setSize(1);
        joints[0]["name"] = "test_shoulder";
        joints[0]["motor_id"] = static_cast<int>(0x144);
        joints[0]["protocol"] = "MT";
        joints[0]["can_device"] = "fake0";
        joints[0]["control_mode"] = "position";
        joints[0]["position_scale"] = 0.1;
        joints[0]["velocity_scale"] = 0.1;
        joints[0]["direction_sign"] = 1.0;
        joints[0]["zero_offset_rad"] = zeroOffsetRad;
        return joints;
    }

    static XmlRpc::XmlRpcValue makeSingleDamiaoVelocityJoint(double directionSign = 1.0)
    {
        XmlRpc::XmlRpcValue joints;
        joints.setSize(1);
        joints[0]["name"] = "test_track";
        joints[0]["motor_id"] = 0x01;
        joints[0]["protocol"] = "DM";
        joints[0]["can_device"] = "fake0";
        joints[0]["control_mode"] = "velocity";
        joints[0]["direction_sign"] = directionSign;
        return joints;
    }

    static XmlRpc::XmlRpcValue makeAliasedMtJoints()
    {
        XmlRpc::XmlRpcValue joints;
        joints.setSize(2);

        joints[0]["name"] = "test_wheel_a";
        joints[0]["motor_id"] = static_cast<int>(0x141);
        joints[0]["protocol"] = "MT";
        joints[0]["can_device"] = "fake0";
        joints[0]["control_mode"] = "velocity";

        joints[1]["name"] = "test_wheel_b";
        joints[1]["motor_id"] = static_cast<int>(0x041);
        joints[1]["protocol"] = "MT";
        joints[1]["can_device"] = "fake0";
        joints[1]["control_mode"] = "velocity";

        return joints;
    }

    static void setPositionLimits(ros::NodeHandle &pnh,
                                  const std::string &jointName,
                                  double minPosition,
                                  double maxPosition)
    {
        const std::string prefix = "joint_limits/" + jointName + "/";
        pnh.setParam(prefix + "has_position_limits", true);
        pnh.setParam(prefix + "min_position", minPosition);
        pnh.setParam(prefix + "max_position", maxPosition);
    }

    static int32_t rawFromPprRadians(double valueRad)
    {
        return static_cast<int32_t>(std::llround(valueRad / (2.0 * M_PI / 65536.0)));
    }

    static void setFreshEnabledFeedback(FakeDeviceManager &dm,
                                        const std::string &device,
                                        CanType protocol,
                                        MotorID motorId,
                                        bool enabled)
    {
        dm.protocol()->setEnabledState(enabled);
        const auto key = can_driver::MakeAxisKey(device, protocol, motorId);
        const auto nowNs = can_driver::SharedDriverSteadyNowNs();
        dm.sharedState()->mutateAxisFeedback(
            key,
            [enabled, nowNs](can_driver::SharedDriverState::AxisFeedbackState *feedback) {
                feedback->feedbackSeen = true;
                feedback->enabled = enabled;
                feedback->enabledValid = true;
                feedback->lastRxSteadyNs = nowNs;
                feedback->lastValidStateSteadyNs = nowNs;
            });
    }

    static std::string uniqueNs(const std::string &base)
    {
        static std::atomic<int> seq{0};
        return "/" + base + "_" + std::to_string(seq.fetch_add(1));
    }

    static std::string uniqueTempFile(const std::string &base)
    {
        static std::atomic<int> seq{0};
        return (std::filesystem::temp_directory_path() /
                (base + "_" + std::to_string(seq.fetch_add(1)) + ".txt"))
            .string();
    }

    static void enterRunning(CanDriverHW &hw)
    {
        auto &coordinator = hw.operationalCoordinator();
        const auto initResult = coordinator.RequestInit("fake0", false);
        ASSERT_TRUE(initResult.ok) << initResult.message;

        const auto enableResult = coordinator.RequestEnable();
        ASSERT_TRUE(enableResult.ok) << enableResult.message;

        const auto releaseResult = coordinator.RequestRelease();
        ASSERT_TRUE(releaseResult.ok) << releaseResult.message;
    }
};

TEST_F(CanDriverHWSmokeTest, InitAndDirectWriteUsesProtocol)
{
    auto fakeDm = std::make_shared<FakeDeviceManager>();
    CanDriverHW hw(fakeDm);

    ros::NodeHandle nh;
    ros::NodeHandle pnh(uniqueNs("can_driver_hw_smoke_write"));

    pnh.setParam("joints", makeSingleVelocityJoint());
    pnh.setParam("debug_bypass_ros_control", true);
    pnh.setParam("motor_state_period_sec", 0.2);

    ASSERT_TRUE(hw.init(nh, pnh));
    enterRunning(hw);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    const std::string cmdTopic = pnh.resolveName("motor/test_wheel/cmd_velocity");
    ros::Publisher pub = nh.advertise<std_msgs::Float64>(cmdTopic, 1);

    for (int i = 0; i < 20 && pub.getNumSubscribers() == 0; ++i) {
        ros::Duration(0.01).sleep();
    }

    std_msgs::Float64 msg;
    msg.data = 1.2;
    pub.publish(msg);
    ros::Duration(0.05).sleep();

    hw.write(ros::Time::now(), ros::Duration(0.01));

    EXPECT_EQ(fakeDm->protocol()->velocityCalls(), 1);
    EXPECT_EQ(fakeDm->protocol()->lastVelocityMotor(), 0x141u);
    EXPECT_EQ(fakeDm->protocol()->lastVelocity(), 12);

    spinner.stop();
}

TEST_F(CanDriverHWSmokeTest, InitRejectsAliasedProtocolNodeIdsOnSameBus)
{
    auto fakeDm = std::make_shared<FakeDeviceManager>();
    CanDriverHW hw(fakeDm);

    ros::NodeHandle nh;
    ros::NodeHandle pnh(uniqueNs("can_driver_hw_smoke_alias"));

    pnh.setParam("joints", makeAliasedMtJoints());

    EXPECT_FALSE(hw.init(nh, pnh));
}

TEST_F(CanDriverHWSmokeTest, InitDefersDeviceActivationUntilLifecycleInit)
{
    auto fakeDm = std::make_shared<FakeDeviceManager>();
    CanDriverHW hw(fakeDm);

    ros::NodeHandle nh;
    ros::NodeHandle pnh(uniqueNs("can_driver_hw_smoke_deferred_activation"));

    pnh.setParam("joints", makeSingleVelocityJoint());
    pnh.setParam("motor_state_period_sec", 0.2);
    pnh.setParam("startup_probe_query_hz", 2.0);
    pnh.setParam("motor_query_hz", 20.0);

    ASSERT_TRUE(hw.init(nh, pnh));
    EXPECT_EQ(fakeDm->ensureTransportCalls(), 0);
    EXPECT_EQ(fakeDm->ensureProtocolCalls(), 0);
    EXPECT_EQ(fakeDm->initDeviceCalls(), 0);
    EXPECT_EQ(fakeDm->deviceCount(), 0u);

    const auto initResult = hw.operationalCoordinator().RequestInit("fake0", false);
    ASSERT_TRUE(initResult.ok) << initResult.message;
    EXPECT_EQ(fakeDm->ensureTransportCalls(), 0);
    EXPECT_EQ(fakeDm->ensureProtocolCalls(), 0);
    EXPECT_EQ(fakeDm->initDeviceCalls(), 1);
    EXPECT_EQ(fakeDm->deviceCount(), 1u);

    const auto deviceRefreshCalls = fakeDm->deviceRefreshRateHzCalls();
    ASSERT_GE(deviceRefreshCalls.size(), 4u);
    EXPECT_EQ(deviceRefreshCalls.front().first, "fake0");
    EXPECT_DOUBLE_EQ(deviceRefreshCalls.front().second, 2.0);
    EXPECT_EQ(deviceRefreshCalls[1].first, "fake0");
    EXPECT_DOUBLE_EQ(deviceRefreshCalls[1].second, 0.0);
    EXPECT_EQ(deviceRefreshCalls[2].first, "fake0");
    EXPECT_DOUBLE_EQ(deviceRefreshCalls[2].second, 20.0);
    EXPECT_EQ(deviceRefreshCalls.back().first, "fake0");
    EXPECT_DOUBLE_EQ(deviceRefreshCalls.back().second, 0.0);

    const auto refreshCalls = fakeDm->refreshRateHzCalls();
    ASSERT_FALSE(refreshCalls.empty());
    EXPECT_DOUBLE_EQ(refreshCalls.back(), 20.0);
}

TEST_F(CanDriverHWSmokeTest, DamiaoVelocityInitAcceptsStartupFeedbackWithoutPosition)
{
    auto fakeDm = std::make_shared<FakeDeviceManager>();
    fakeDm->setVelocityOnlyFeedbackOnInit(true);
    CanDriverHW hw(fakeDm);

    ros::NodeHandle nh;
    ros::NodeHandle pnh(uniqueNs("can_driver_hw_smoke_dm_velocity_startup"));

    pnh.setParam("joints", makeSingleDamiaoVelocityJoint());
    pnh.setParam("motor_state_period_sec", 0.2);
    pnh.setParam("startup_probe_query_hz", 2.0);
    pnh.setParam("motor_query_hz", 20.0);

    ASSERT_TRUE(hw.init(nh, pnh));

    const auto initResult = hw.operationalCoordinator().RequestInit("fake0", false);
    EXPECT_TRUE(initResult.ok) << initResult.message;
    EXPECT_EQ(hw.lifecycleMode(), can_driver::SystemOpMode::Standby);
    EXPECT_EQ(fakeDm->shutdownDeviceCalls(), 0);
}

TEST_F(CanDriverHWSmokeTest, InitFailsFastWhenStartupFeedbackNeverArrives)
{
    auto fakeDm = std::make_shared<FakeDeviceManager>();
    fakeDm->setSeedFeedbackOnInit(false);
    CanDriverHW hw(fakeDm);

    ros::NodeHandle nh;
    ros::NodeHandle pnh(uniqueNs("can_driver_hw_smoke_startup_timeout"));

    pnh.setParam("joints", makeSingleVelocityJoint());
    pnh.setParam("motor_state_period_sec", 0.2);
    pnh.setParam("startup_position_sync_timeout_sec", 0.05);
    pnh.setParam("startup_probe_query_hz", 2.0);
    pnh.setParam("motor_query_hz", 20.0);

    ASSERT_TRUE(hw.init(nh, pnh));

    const auto initResult = hw.operationalCoordinator().RequestInit("fake0", false);
    EXPECT_FALSE(initResult.ok);
    EXPECT_EQ(fakeDm->shutdownDeviceCalls(), 1);
    EXPECT_EQ(fakeDm->lastShutdownDevice(), "fake0");

    const auto deviceRefreshCalls = fakeDm->deviceRefreshRateHzCalls();
    ASSERT_FALSE(deviceRefreshCalls.empty());
    EXPECT_EQ(deviceRefreshCalls.front().first, "fake0");
    EXPECT_DOUBLE_EQ(deviceRefreshCalls.front().second, 2.0);
}

TEST_F(CanDriverHWSmokeTest, InitAppliesLocalZeroOffsetBeforeStartupLimitCheck)
{
    auto fakeDm = std::make_shared<FakeDeviceManager>();
    fakeDm->protocol()->setFeedbackPosition(420);
    CanDriverHW hw(fakeDm);

    ros::NodeHandle nh;
    ros::NodeHandle pnh(uniqueNs("can_driver_hw_smoke_startup_zero_offset"));

    pnh.setParam("joints", makeSingleMtPositionJoint(-42.0));
    pnh.setParam("motor_state_period_sec", 0.2);
    pnh.setParam("startup_probe_query_hz", 2.0);
    pnh.setParam("motor_query_hz", 20.0);
    setPositionLimits(pnh, "test_shoulder", -1.0, 1.0);

    ASSERT_TRUE(hw.init(nh, pnh));

    const auto initResult = hw.operationalCoordinator().RequestInit("fake0", false);
    EXPECT_TRUE(initResult.ok) << initResult.message;
    EXPECT_EQ(fakeDm->shutdownDeviceCalls(), 0);
}

TEST_F(CanDriverHWSmokeTest, MotorCommandServiceEnable)
{
    auto fakeDm = std::make_shared<FakeDeviceManager>();
    CanDriverHW hw(fakeDm);

    ros::NodeHandle nh;
    ros::NodeHandle pnh(uniqueNs("can_driver_hw_smoke_service"));

    pnh.setParam("joints", makeSingleVelocityJoint());
    pnh.setParam("motor_state_period_sec", 0.2);

    ASSERT_TRUE(hw.init(nh, pnh));

    ros::AsyncSpinner spinner(1);
    spinner.start();

    const std::string srvName = pnh.resolveName("motor_command");
    ros::ServiceClient client = nh.serviceClient<can_driver::MotorCommand>(srvName);
    ASSERT_TRUE(client.waitForExistence(ros::Duration(1.0)));

    can_driver::MotorCommand srv;
    srv.request.motor_id = 0x141;
    srv.request.command = can_driver::MotorCommand::Request::CMD_ENABLE;
    srv.request.value = 0.0;

    ASSERT_TRUE(client.call(srv));
    EXPECT_FALSE(srv.response.success);
    EXPECT_EQ(srv.response.message, "Driver inactive.");
    EXPECT_EQ(fakeDm->protocol()->enableCalls(), 0);

    const auto initResult = hw.operationalCoordinator().RequestInit("fake0", false);
    ASSERT_TRUE(initResult.ok) << initResult.message;

    ASSERT_TRUE(client.call(srv));
    EXPECT_TRUE(srv.response.success);
    EXPECT_EQ(fakeDm->protocol()->enableCalls(), 1);
    EXPECT_EQ(fakeDm->protocol()->lastEnableMotor(), 0x141u);

    spinner.stop();
}

TEST_F(CanDriverHWSmokeTest, RecoverServiceRejectsPerMotorLegacyRequest)
{
    auto fakeDm = std::make_shared<FakeDeviceManager>();
    CanDriverHW hw(fakeDm);

    ros::NodeHandle nh;
    ros::NodeHandle pnh(uniqueNs("can_driver_hw_smoke_recover_service"));

    pnh.setParam("joints", makeSingleVelocityJoint());
    pnh.setParam("motor_state_period_sec", 0.2);

    ASSERT_TRUE(hw.init(nh, pnh));

    can_driver::OperationalCoordinator::DriverOps ops;
    ops.recover_all = []() {
        return can_driver::OperationalCoordinator::Result{true, "recovered"};
    };
    hw.operationalCoordinator().SetDriverOps(std::move(ops));
    hw.operationalCoordinator().SetFaulted();

    LifecycleServiceGateway gateway(pnh, &hw.operationalCoordinator());

    ros::AsyncSpinner spinner(1);
    spinner.start();

    const std::string srvName = pnh.resolveName("recover");
    ros::ServiceClient client = nh.serviceClient<can_driver::Recover>(srvName);
    ASSERT_TRUE(client.waitForExistence(ros::Duration(1.0)));

    can_driver::Recover srv;
    srv.request.motor_id = 0x141;

    ASSERT_TRUE(client.call(srv));
    EXPECT_FALSE(srv.response.success);
    EXPECT_EQ(srv.response.message,
              "per-motor recover has been removed; use motor_id=65535 for global recover");

    spinner.stop();
}

TEST_F(CanDriverHWSmokeTest, LifecycleServicesExposeCanonicalMessages)
{
    auto fakeDm = std::make_shared<FakeDeviceManager>();
    CanDriverHW hw(fakeDm);

    ros::NodeHandle nh;
    ros::NodeHandle pnh(uniqueNs("can_driver_hw_smoke_lifecycle_messages"));

    pnh.setParam("joints", makeSingleVelocityJoint());
    pnh.setParam("motor_state_period_sec", 0.2);

    ASSERT_TRUE(hw.init(nh, pnh));

    can_driver::OperationalCoordinator::DriverOps ops;
    ops.init_device = [](const std::string &, bool) {
        return can_driver::OperationalCoordinator::Result{true, "driver init detail"};
    };
    ops.enable_all = []() {
        return can_driver::OperationalCoordinator::Result{true, "driver enable detail"};
    };
    ops.disable_all = []() {
        return can_driver::OperationalCoordinator::Result{true, "OK"};
    };
    ops.halt_all = []() {
        return can_driver::OperationalCoordinator::Result{true, "OK"};
    };
    ops.recover_all = []() {
        return can_driver::OperationalCoordinator::Result{true, "driver recover detail"};
    };
    ops.shutdown_all = [](bool) {
        return can_driver::OperationalCoordinator::Result{true, "driver shutdown detail"};
    };
    ops.motion_healthy = [](std::string *) {
        return true;
    };
    ops.any_fault_active = []() {
        return false;
    };
    ops.hold_commands = []() {};
    ops.arm_fresh_command_latch = []() {};

    auto &coordinator = hw.operationalCoordinator();
    coordinator.SetDriverOps(std::move(ops));
    coordinator.SetConfigured();

    LifecycleServiceGateway gateway(pnh, &coordinator);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::ServiceClient initClient = nh.serviceClient<can_driver::Init>(pnh.resolveName("init"));
    ros::ServiceClient enableClient =
        nh.serviceClient<std_srvs::Trigger>(pnh.resolveName("enable"));
    ros::ServiceClient disableClient =
        nh.serviceClient<std_srvs::Trigger>(pnh.resolveName("disable"));
    ros::ServiceClient recoverClient =
        nh.serviceClient<can_driver::Recover>(pnh.resolveName("recover"));
    ros::ServiceClient resumeClient =
        nh.serviceClient<std_srvs::Trigger>(pnh.resolveName("resume"));
    ros::ServiceClient haltClient =
        nh.serviceClient<std_srvs::Trigger>(pnh.resolveName("halt"));
    ros::ServiceClient shutdownClient =
        nh.serviceClient<can_driver::Shutdown>(pnh.resolveName("shutdown"));

    ASSERT_TRUE(initClient.waitForExistence(ros::Duration(1.0)));
    ASSERT_TRUE(enableClient.waitForExistence(ros::Duration(1.0)));
    ASSERT_TRUE(disableClient.waitForExistence(ros::Duration(1.0)));
    ASSERT_TRUE(recoverClient.waitForExistence(ros::Duration(1.0)));
    ASSERT_TRUE(resumeClient.waitForExistence(ros::Duration(1.0)));
    ASSERT_TRUE(haltClient.waitForExistence(ros::Duration(1.0)));
    ASSERT_TRUE(shutdownClient.waitForExistence(ros::Duration(1.0)));

    can_driver::Init initSrv;
    initSrv.request.device = "fake0";
    initSrv.request.loopback = false;
    ASSERT_TRUE(initClient.call(initSrv));
    EXPECT_TRUE(initSrv.response.success);
    EXPECT_EQ(initSrv.response.message, "initialized (standby)");

    can_driver::Init initAgainSrv;
    initAgainSrv.request.device = "fake0";
    initAgainSrv.request.loopback = false;
    ASSERT_TRUE(initClient.call(initAgainSrv));
    EXPECT_TRUE(initAgainSrv.response.success);
    EXPECT_EQ(initAgainSrv.response.message, "already initialized");

    std_srvs::Trigger enableSrv;
    ASSERT_TRUE(enableClient.call(enableSrv));
    EXPECT_TRUE(enableSrv.response.success);
    EXPECT_EQ(enableSrv.response.message, "enabled (armed)");

    std_srvs::Trigger resumeSrv;
    ASSERT_TRUE(resumeClient.call(resumeSrv));
    EXPECT_TRUE(resumeSrv.response.success);
    EXPECT_EQ(resumeSrv.response.message, "resumed");

    std_srvs::Trigger resumeAgainSrv;
    ASSERT_TRUE(resumeClient.call(resumeAgainSrv));
    EXPECT_TRUE(resumeAgainSrv.response.success);
    EXPECT_EQ(resumeAgainSrv.response.message, "already running");

    std_srvs::Trigger haltSrv;
    ASSERT_TRUE(haltClient.call(haltSrv));
    EXPECT_TRUE(haltSrv.response.success);
    EXPECT_EQ(haltSrv.response.message, "halted");

    std_srvs::Trigger haltAgainSrv;
    ASSERT_TRUE(haltClient.call(haltAgainSrv));
    EXPECT_TRUE(haltAgainSrv.response.success);
    EXPECT_EQ(haltAgainSrv.response.message, "already halted");

    std_srvs::Trigger disableSrv;
    ASSERT_TRUE(disableClient.call(disableSrv));
    EXPECT_TRUE(disableSrv.response.success);
    EXPECT_EQ(disableSrv.response.message, "disabled (standby)");

    std_srvs::Trigger enableAgainSrv;
    ASSERT_TRUE(enableClient.call(enableAgainSrv));
    EXPECT_TRUE(enableAgainSrv.response.success);
    EXPECT_EQ(enableAgainSrv.response.message, "enabled (armed)");

    ASSERT_TRUE(enableClient.call(enableAgainSrv));
    EXPECT_TRUE(enableAgainSrv.response.success);
    EXPECT_EQ(enableAgainSrv.response.message, "already enabled");

    coordinator.SetFaulted();

    can_driver::Recover recoverSrv;
    recoverSrv.request.motor_id = 0xFFFFu;
    ASSERT_TRUE(recoverClient.call(recoverSrv));
    EXPECT_TRUE(recoverSrv.response.success);
    EXPECT_EQ(recoverSrv.response.message, "recovered (standby)");

    can_driver::Shutdown shutdownSrv;
    shutdownSrv.request.force = false;
    ASSERT_TRUE(shutdownClient.call(shutdownSrv));
    EXPECT_TRUE(shutdownSrv.response.success);
    EXPECT_EQ(shutdownSrv.response.message, "communication stopped; call ~/init first");

    spinner.stop();
}

TEST_F(CanDriverHWSmokeTest, LifecycleStateTopicTracksCoordinatorMode)
{
    auto fakeDm = std::make_shared<FakeDeviceManager>();
    CanDriverHW hw(fakeDm);

    ros::NodeHandle nh;
    ros::NodeHandle pnh(uniqueNs("can_driver_hw_smoke_lifecycle_topic"));

    pnh.setParam("joints", makeSingleVelocityJoint());
    pnh.setParam("motor_state_period_sec", 0.2);

    ASSERT_TRUE(hw.init(nh, pnh));

    std::mutex stateMutex;
    std::string latestState;
    ros::Subscriber stateSub = nh.subscribe<std_msgs::String>(
        pnh.resolveName("lifecycle_state"), 1,
        [&stateMutex, &latestState](const std_msgs::String::ConstPtr &msg) {
            std::lock_guard<std::mutex> lock(stateMutex);
            latestState = msg->data;
        });

    ros::AsyncSpinner spinner(1);
    spinner.start();

    auto waitForState = [&stateMutex, &latestState](const std::string &expected) {
        for (int i = 0; i < 50; ++i) {
            {
                std::lock_guard<std::mutex> lock(stateMutex);
                if (latestState == expected) {
                    return true;
                }
            }
            ros::Duration(0.02).sleep();
        }
        return false;
    };

    ASSERT_TRUE(waitForState("Configured"));

    const auto initResult = hw.operationalCoordinator().RequestInit("fake0", false);
    ASSERT_TRUE(initResult.ok) << initResult.message;
    ASSERT_TRUE(waitForState("Standby"));

    const auto enableResult = hw.operationalCoordinator().RequestEnable();
    ASSERT_TRUE(enableResult.ok) << enableResult.message;
    ASSERT_TRUE(waitForState("Armed"));

    const auto releaseResult = hw.operationalCoordinator().RequestRelease();
    ASSERT_TRUE(releaseResult.ok) << releaseResult.message;
    ASSERT_TRUE(waitForState("Running"));

    const auto haltResult = hw.operationalCoordinator().RequestHalt();
    ASSERT_TRUE(haltResult.ok) << haltResult.message;
    ASSERT_TRUE(waitForState("Armed"));

    const auto shutdownResult = hw.operationalCoordinator().RequestShutdown(false);
    ASSERT_TRUE(shutdownResult.ok) << shutdownResult.message;
    ASSERT_TRUE(waitForState("Configured"));

    spinner.stop();
}

TEST_F(CanDriverHWSmokeTest, InitRegistersRosControlInterfacesForJointMode)
{
    {
        auto fakeDm = std::make_shared<FakeDeviceManager>();
        CanDriverHW hw(fakeDm);

        ros::NodeHandle nh;
        ros::NodeHandle pnh(uniqueNs("can_driver_hw_smoke_velocity_interfaces"));

        pnh.setParam("joints", makeSingleVelocityJoint());
        ASSERT_TRUE(hw.init(nh, pnh));

        auto *stateIface = hw.get<hardware_interface::JointStateInterface>();
        auto *velIface = hw.get<hardware_interface::VelocityJointInterface>();
        auto *posIface = hw.get<hardware_interface::PositionJointInterface>();

        ASSERT_NE(stateIface, nullptr);
        ASSERT_NE(velIface, nullptr);
        ASSERT_NE(posIface, nullptr);

        EXPECT_NO_THROW(stateIface->getHandle("test_wheel"));
        EXPECT_NO_THROW(velIface->getHandle("test_wheel"));
        EXPECT_THROW(posIface->getHandle("test_wheel"),
                     hardware_interface::HardwareInterfaceException);
    }

    {
        auto fakeDm = std::make_shared<FakeDeviceManager>();
        fakeDm->protocol()->setFeedbackPosition(512);
        CanDriverHW hw(fakeDm);

        ros::NodeHandle nh;
        ros::NodeHandle pnh(uniqueNs("can_driver_hw_smoke_position_interfaces"));

        pnh.setParam("joints", makeSingleCspJoint());
        ASSERT_TRUE(hw.init(nh, pnh));

        auto *stateIface = hw.get<hardware_interface::JointStateInterface>();
        auto *velIface = hw.get<hardware_interface::VelocityJointInterface>();
        auto *posIface = hw.get<hardware_interface::PositionJointInterface>();

        ASSERT_NE(stateIface, nullptr);
        ASSERT_NE(velIface, nullptr);
        ASSERT_NE(posIface, nullptr);

        EXPECT_NO_THROW(stateIface->getHandle("test_arm"));
        EXPECT_NO_THROW(posIface->getHandle("test_arm"));
        EXPECT_THROW(velIface->getHandle("test_arm"),
                     hardware_interface::HardwareInterfaceException);
    }
}

TEST_F(CanDriverHWSmokeTest, InitCanDisableRosEndpointsForEmbeddedUse)
{
    auto fakeDm = std::make_shared<FakeDeviceManager>();
    CanDriverHW hw(fakeDm);

    ros::NodeHandle nh;
    ros::NodeHandle pnh(uniqueNs("can_driver_hw_smoke_embedded_mode"));

    pnh.setParam("joints", makeSingleVelocityJoint());
    pnh.setParam("motor_state_period_sec", 0.05);

    CanDriverHW::InitOptions options;
    options.enable_ros_endpoints = false;
    ASSERT_TRUE(hw.init(nh, pnh, options));

    ros::AsyncSpinner spinner(1);
    spinner.start();

    EXPECT_FALSE(ros::service::exists(pnh.resolveName("motor_command"), false));
    EXPECT_FALSE(ros::service::exists(pnh.resolveName("set_zero_limit"), false));

    ros::Publisher pub = nh.advertise<std_msgs::Float64>(
        pnh.resolveName("motor/test_wheel/cmd_velocity"), 1);
    ros::Duration(0.05).sleep();
    EXPECT_EQ(pub.getNumSubscribers(), 0u);

    spinner.stop();
}

TEST_F(CanDriverHWSmokeTest, WriteAutoStopOnFaultAndBlockMotion)
{
    auto fakeDm = std::make_shared<FakeDeviceManager>();
    CanDriverHW hw(fakeDm);

    ros::NodeHandle nh;
    ros::NodeHandle pnh(uniqueNs("can_driver_hw_smoke_fault"));

    pnh.setParam("joints", makeSingleVelocityJoint());
    pnh.setParam("debug_bypass_ros_control", true);
    pnh.setParam("motor_state_period_sec", 0.2);
    pnh.setParam("safety_stop_on_fault", true);
    pnh.setParam("safety_require_enabled_for_motion", false);

    ASSERT_TRUE(hw.init(nh, pnh));
    enterRunning(hw);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    const std::string cmdTopic = pnh.resolveName("motor/test_wheel/cmd_velocity");
    ros::Publisher pub = nh.advertise<std_msgs::Float64>(cmdTopic, 1);
    for (int i = 0; i < 20 && pub.getNumSubscribers() == 0; ++i) {
        ros::Duration(0.01).sleep();
    }

    std_msgs::Float64 msg;
    msg.data = 2.0;
    pub.publish(msg);
    ros::Duration(0.05).sleep();

    fakeDm->protocol()->setFault(true);
    hw.write(ros::Time::now(), ros::Duration(0.01));
    hw.write(ros::Time::now(), ros::Duration(0.01));

    EXPECT_EQ(fakeDm->protocol()->stopCalls(), 1);
    EXPECT_EQ(fakeDm->protocol()->velocityCalls(), 0);

    spinner.stop();
}

TEST_F(CanDriverHWSmokeTest, WriteUsesFreshSharedFeedbackInsteadOfProtocolCacheForSafetyGates)
{
    auto fakeDm = std::make_shared<FakeDeviceManager>();
    CanDriverHW hw(fakeDm);

    ros::NodeHandle nh;
    ros::NodeHandle pnh(uniqueNs("can_driver_hw_smoke_shared_write_gate"));

    pnh.setParam("joints", makeSingleVelocityJoint());
    pnh.setParam("debug_bypass_ros_control", true);
    pnh.setParam("motor_state_period_sec", 0.2);
    pnh.setParam("safety_stop_on_fault", true);
    pnh.setParam("safety_require_enabled_for_motion", true);

    ASSERT_TRUE(hw.init(nh, pnh));
    enterRunning(hw);
    fakeDm->setSyncSharedFeedbackFromProtocol(false);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    const std::string cmdTopic = pnh.resolveName("motor/test_wheel/cmd_velocity");
    ros::Publisher pub = nh.advertise<std_msgs::Float64>(cmdTopic, 1);
    for (int i = 0; i < 20 && pub.getNumSubscribers() == 0; ++i) {
        ros::Duration(0.01).sleep();
    }

    fakeDm->protocol()->setEnabledState(false);
    fakeDm->protocol()->setFault(true);
    fakeDm->sharedState()->mutateAxisFeedback(
        can_driver::MakeAxisKey("fake0", CanType::MT, static_cast<MotorID>(0x141)),
        [](can_driver::SharedDriverState::AxisFeedbackState *feedback) {
            feedback->feedbackSeen = true;
            feedback->enabled = true;
            feedback->fault = false;
            feedback->lastRxSteadyNs = can_driver::SharedDriverSteadyNowNs();
            feedback->lastValidStateSteadyNs = feedback->lastRxSteadyNs;
        });

    std_msgs::Float64 msg;
    msg.data = 2.0;
    pub.publish(msg);
    ros::Duration(0.05).sleep();

    hw.write(ros::Time::now(), ros::Duration(0.01));

    EXPECT_EQ(fakeDm->protocol()->stopCalls(), 0);
    EXPECT_EQ(fakeDm->protocol()->velocityCalls(), 1);
    EXPECT_EQ(fakeDm->protocol()->lastVelocityMotor(), 0x141u);
    EXPECT_EQ(fakeDm->protocol()->lastVelocity(), 20);

    spinner.stop();
}

TEST_F(CanDriverHWSmokeTest, WriteFallsBackToProtocolSafetyStateWhenSharedFeedbackStale)
{
    auto fakeDm = std::make_shared<FakeDeviceManager>();
    CanDriverHW hw(fakeDm);

    ros::NodeHandle nh;
    ros::NodeHandle pnh(uniqueNs("can_driver_hw_smoke_stale_shared_write_gate"));

    pnh.setParam("joints", makeSingleVelocityJoint());
    pnh.setParam("debug_bypass_ros_control", true);
    pnh.setParam("motor_state_period_sec", 0.2);
    pnh.setParam("safety_stop_on_fault", true);
    pnh.setParam("safety_require_enabled_for_motion", true);

    ASSERT_TRUE(hw.init(nh, pnh));
    enterRunning(hw);
    fakeDm->setSyncSharedFeedbackFromProtocol(false);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    const std::string cmdTopic = pnh.resolveName("motor/test_wheel/cmd_velocity");
    ros::Publisher pub = nh.advertise<std_msgs::Float64>(cmdTopic, 1);
    for (int i = 0; i < 20 && pub.getNumSubscribers() == 0; ++i) {
        ros::Duration(0.01).sleep();
    }

    fakeDm->protocol()->setEnabledState(false);
    fakeDm->protocol()->setFault(true);
    fakeDm->sharedState()->mutateAxisFeedback(
        can_driver::MakeAxisKey("fake0", CanType::MT, static_cast<MotorID>(0x141)),
        [](can_driver::SharedDriverState::AxisFeedbackState *feedback) {
            feedback->feedbackSeen = true;
            feedback->enabled = true;
            feedback->fault = false;
            feedback->lastRxSteadyNs = can_driver::SharedDriverSteadyNowNs() - 2000000000LL;
            feedback->lastValidStateSteadyNs = feedback->lastRxSteadyNs;
        });

    std_msgs::Float64 msg;
    msg.data = 2.0;
    pub.publish(msg);
    ros::Duration(0.05).sleep();

    hw.write(ros::Time::now(), ros::Duration(0.01));

    EXPECT_EQ(fakeDm->protocol()->stopCalls(), 1);
    EXPECT_EQ(fakeDm->protocol()->velocityCalls(), 0);

    spinner.stop();
}

TEST_F(CanDriverHWSmokeTest, DisconnectFaultsSystemAndRecoverClearsStaleDirectCommand)
{
    auto fakeDm = std::make_shared<FakeDeviceManager>();
    CanDriverHW hw(fakeDm);

    ros::NodeHandle nh;
    ros::NodeHandle pnh(uniqueNs("can_driver_hw_smoke_recover_hold"));

    pnh.setParam("joints", makeSingleVelocityJoint());
    pnh.setParam("debug_bypass_ros_control", true);
    pnh.setParam("motor_state_period_sec", 0.2);
    pnh.setParam("safety_stop_on_fault", false);
    pnh.setParam("safety_require_enabled_for_motion", false);
    pnh.setParam("safety_hold_after_device_recover", true);

    ASSERT_TRUE(hw.init(nh, pnh));
    enterRunning(hw);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    const std::string cmdTopic = pnh.resolveName("motor/test_wheel/cmd_velocity");
    ros::Publisher pub = nh.advertise<std_msgs::Float64>(cmdTopic, 1);
    for (int i = 0; i < 20 && pub.getNumSubscribers() == 0; ++i) {
        ros::Duration(0.01).sleep();
    }

    std_msgs::Float64 msg;
    msg.data = 1.0;
    pub.publish(msg);
    ros::Duration(0.05).sleep();
    hw.write(ros::Time::now(), ros::Duration(0.01));
    EXPECT_EQ(fakeDm->protocol()->velocityCalls(), 1);

    fakeDm->setReady(false);
    msg.data = 3.0;
    pub.publish(msg);
    ros::Duration(0.05).sleep();
    hw.write(ros::Time::now(), ros::Duration(0.01));
    EXPECT_EQ(fakeDm->protocol()->velocityCalls(), 1);
    EXPECT_EQ(hw.lifecycleMode(), can_driver::SystemOpMode::Faulted);

    fakeDm->setReady(true);
    const auto recoverResult = hw.operationalCoordinator().RequestRecover();
    ASSERT_TRUE(recoverResult.ok) << recoverResult.message;
    EXPECT_EQ(hw.lifecycleMode(), can_driver::SystemOpMode::Standby);
    EXPECT_EQ(fakeDm->initDeviceCalls(), 2);
    EXPECT_EQ(fakeDm->protocol()->resetFaultCalls(), 1);

    const auto enableResult = hw.operationalCoordinator().RequestEnable();
    ASSERT_TRUE(enableResult.ok) << enableResult.message;
    const auto releaseResult = hw.operationalCoordinator().RequestRelease();
    ASSERT_TRUE(releaseResult.ok) << releaseResult.message;

    // 掉线前积累的 direct 命令应被清空；recover + enable + resume 后仍需 fresh 命令。
    hw.write(ros::Time::now(), ros::Duration(0.01));
    EXPECT_EQ(fakeDm->protocol()->velocityCalls(), 1);

    msg.data = 4.0;
    pub.publish(msg);
    ros::Duration(0.05).sleep();
    hw.write(ros::Time::now(), ros::Duration(0.01));
    EXPECT_EQ(fakeDm->protocol()->velocityCalls(), 2);
    EXPECT_EQ(fakeDm->protocol()->lastVelocity(), 40);

    spinner.stop();
}

TEST_F(CanDriverHWSmokeTest, InitCspJointSetsModeAndPublishesRawFeedbackFromPprConfig)
{
    auto fakeDm = std::make_shared<FakeDeviceManager>();
    fakeDm->protocol()->setFeedbackPosition(16384);
    fakeDm->protocol()->setFeedbackVelocity(512);

    CanDriverHW hw(fakeDm);

    ros::NodeHandle nh;
    ros::NodeHandle pnh(uniqueNs("can_driver_hw_smoke_csp_init"));

    pnh.setParam("joints", makeSingleCspJoint());
    pnh.setParam("motor_state_period_sec", 0.05);

    ASSERT_TRUE(hw.init(nh, pnh));
    EXPECT_EQ(fakeDm->protocol()->setModeCalls(), 0);

    std::mutex stateMutex;
    can_driver::MotorState latestState;
    bool gotState = false;
    ros::Subscriber stateSub = nh.subscribe<can_driver::MotorState>(
        pnh.resolveName("motor_states"), 1,
        [&stateMutex, &latestState, &gotState](const can_driver::MotorState::ConstPtr &msg) {
            std::lock_guard<std::mutex> lock(stateMutex);
            latestState = *msg;
            gotState = true;
        });

    ros::AsyncSpinner spinner(1);
    spinner.start();

    const auto initResult = hw.operationalCoordinator().RequestInit("fake0", false);
    ASSERT_TRUE(initResult.ok) << initResult.message;
    EXPECT_EQ(fakeDm->protocol()->setModeCalls(), 1);
    EXPECT_EQ(fakeDm->protocol()->lastModeMotor(), 0x05u);
    EXPECT_EQ(fakeDm->protocol()->lastMode(), CanProtocol::MotorMode::CSP);

    for (int i = 0; i < 20 && !gotState; ++i) {
        ros::Duration(0.01).sleep();
    }

    ASSERT_TRUE(gotState);
    EXPECT_EQ(latestState.motor_id, 0x05u);
    EXPECT_EQ(latestState.position, 16384);
    EXPECT_EQ(latestState.velocity, 512);
    EXPECT_EQ(latestState.mode, can_driver::MotorState::MODE_CSP);
    EXPECT_TRUE(latestState.mode_valid);
    EXPECT_TRUE(latestState.status_valid);
    EXPECT_TRUE(latestState.position_valid);
    EXPECT_TRUE(latestState.velocity_valid);
    EXPECT_TRUE(latestState.current_valid);
    EXPECT_TRUE(latestState.feedback_fresh);

    spinner.stop();
}

TEST_F(CanDriverHWSmokeTest, MotorStatesPreferSharedFeedbackOverConfiguredModeAndProtocolCache)
{
    auto fakeDm = std::make_shared<FakeDeviceManager>();
    fakeDm->protocol()->setEnabledState(true);
    fakeDm->protocol()->setFault(false);

    CanDriverHW hw(fakeDm);

    ros::NodeHandle nh;
    ros::NodeHandle pnh(uniqueNs("can_driver_hw_smoke_shared_motor_state"));

    pnh.setParam("joints", makeSingleCspJoint());
    pnh.setParam("motor_state_period_sec", 0.05);

    ASSERT_TRUE(hw.init(nh, pnh));

    std::mutex stateMutex;
    can_driver::MotorState latestState;
    bool sawSharedState = false;
    ros::Subscriber stateSub = nh.subscribe<can_driver::MotorState>(
        pnh.resolveName("motor_states"), 1,
        [&stateMutex, &latestState, &sawSharedState](const can_driver::MotorState::ConstPtr &msg) {
            std::lock_guard<std::mutex> lock(stateMutex);
            latestState = *msg;
            sawSharedState =
                msg->mode_valid && msg->mode == can_driver::MotorState::MODE_POSITION &&
                msg->status_valid && !msg->enabled && msg->fault;
        });

    ros::AsyncSpinner spinner(1);
    spinner.start();

    const auto initResult = hw.operationalCoordinator().RequestInit("fake0", false);
    ASSERT_TRUE(initResult.ok) << initResult.message;

    const auto axisKey = can_driver::MakeAxisKey("fake0", CanType::PP, static_cast<MotorID>(0x05));
    fakeDm->sharedState()->mutateAxisFeedback(
        axisKey,
        [](can_driver::SharedDriverState::AxisFeedbackState *feedback) {
            feedback->feedbackSeen = true;
            feedback->enabled = false;
            feedback->fault = true;
            feedback->mode = CanProtocol::MotorMode::Position;
            feedback->modeValid = true;
            feedback->enabledValid = true;
            feedback->faultValid = true;
            feedback->lastRxSteadyNs = can_driver::SharedDriverSteadyNowNs();
            feedback->lastValidStateSteadyNs = feedback->lastRxSteadyNs;
        });

    for (int i = 0; i < 20 && !sawSharedState; ++i) {
        ros::Duration(0.02).sleep();
    }

    ASSERT_TRUE(sawSharedState);
    EXPECT_EQ(latestState.mode, can_driver::MotorState::MODE_POSITION);
    EXPECT_TRUE(latestState.mode_valid);
    EXPECT_FALSE(latestState.enabled);
    EXPECT_TRUE(latestState.fault);
    EXPECT_TRUE(latestState.status_valid);
    EXPECT_TRUE(latestState.feedback_fresh);

    spinner.stop();
}

TEST_F(CanDriverHWSmokeTest, MotorStatesPublishUnknownWhenFeedbackValidityIsMissingOrStale)
{
    auto fakeDm = std::make_shared<FakeDeviceManager>();

    CanDriverHW hw(fakeDm);

    ros::NodeHandle nh;
    ros::NodeHandle pnh(uniqueNs("can_driver_hw_smoke_invalid_motor_state"));

    pnh.setParam("joints", makeSingleCspJoint());
    pnh.setParam("motor_state_period_sec", 0.05);

    ASSERT_TRUE(hw.init(nh, pnh));

    std::mutex stateMutex;
    can_driver::MotorState latestState;
    bool sawInvalidState = false;
    ros::Subscriber stateSub = nh.subscribe<can_driver::MotorState>(
        pnh.resolveName("motor_states"), 1,
        [&stateMutex, &latestState, &sawInvalidState](const can_driver::MotorState::ConstPtr &msg) {
            std::lock_guard<std::mutex> lock(stateMutex);
            latestState = *msg;
            sawInvalidState = !msg->mode_valid && !msg->status_valid &&
                              msg->position_valid && !msg->velocity_valid &&
                              !msg->current_valid && !msg->feedback_fresh;
        });

    ros::AsyncSpinner spinner(1);
    spinner.start();

    const auto initResult = hw.operationalCoordinator().RequestInit("fake0", false);
    ASSERT_TRUE(initResult.ok) << initResult.message;

    const auto axisKey = can_driver::MakeAxisKey("fake0", CanType::PP, static_cast<MotorID>(0x05));
    fakeDm->sharedState()->mutateAxisFeedback(
        axisKey,
        [](can_driver::SharedDriverState::AxisFeedbackState *feedback) {
            feedback->feedbackSeen = true;
            feedback->position = 2048;
            feedback->velocity = 777;
            feedback->current = 123;
            feedback->positionValid = true;
            feedback->velocityValid = false;
            feedback->currentValid = false;
            feedback->mode = CanProtocol::MotorMode::CSP;
            feedback->modeValid = false;
            feedback->enabled = true;
            feedback->fault = true;
            feedback->enabledValid = false;
            feedback->faultValid = false;
            feedback->lastRxSteadyNs =
                can_driver::SharedDriverSteadyNowNs() - 1000000000LL;
            feedback->lastValidStateSteadyNs = feedback->lastRxSteadyNs;
        });

    for (int i = 0; i < 20 && !sawInvalidState; ++i) {
        ros::Duration(0.02).sleep();
    }

    ASSERT_TRUE(sawInvalidState);
    EXPECT_EQ(latestState.position, 2048);
    EXPECT_TRUE(latestState.position_valid);
    EXPECT_FALSE(latestState.velocity_valid);
    EXPECT_FALSE(latestState.current_valid);
    EXPECT_EQ(latestState.mode, can_driver::MotorState::MODE_UNKNOWN);
    EXPECT_FALSE(latestState.mode_valid);
    EXPECT_FALSE(latestState.status_valid);
    EXPECT_FALSE(latestState.enabled);
    EXPECT_FALSE(latestState.fault);
    EXPECT_FALSE(latestState.feedback_fresh);

    spinner.stop();
}

TEST_F(CanDriverHWSmokeTest, InitFailureRollsBackPreparedDevice)
{
    auto fakeDm = std::make_shared<FakeDeviceManager>();
    fakeDm->protocol()->setModeResult(false);

    CanDriverHW hw(fakeDm);

    ros::NodeHandle nh;
    ros::NodeHandle pnh(uniqueNs("can_driver_hw_smoke_init_rollback"));

    pnh.setParam("joints", makeSingleCspJoint());
    pnh.setParam("motor_state_period_sec", 0.05);

    ASSERT_TRUE(hw.init(nh, pnh));

    auto &coordinator = hw.operationalCoordinator();
    const auto failedInit = coordinator.RequestInit("fake0", false);
    EXPECT_FALSE(failedInit.ok);
    EXPECT_EQ(failedInit.message, "Failed to apply initial modes on fake0");
    EXPECT_EQ(coordinator.mode(), can_driver::SystemOpMode::Configured);
    EXPECT_EQ(fakeDm->shutdownDeviceCalls(), 1);
    EXPECT_EQ(fakeDm->lastShutdownDevice(), "fake0");

    fakeDm->protocol()->setModeResult(true);
    const auto retriedInit = coordinator.RequestInit("fake0", false);
    EXPECT_TRUE(retriedInit.ok) << retriedInit.message;
    EXPECT_EQ(coordinator.mode(), can_driver::SystemOpMode::Standby);
    EXPECT_EQ(fakeDm->shutdownDeviceCalls(), 1);
}

TEST_F(CanDriverHWSmokeTest, CspInitRejectsStartupPositionOutsideConfiguredLimits)
{
    auto fakeDm = std::make_shared<FakeDeviceManager>();
    fakeDm->protocol()->setFeedbackPosition(rawFromPprRadians(1.2));

    CanDriverHW hw(fakeDm);

    ros::NodeHandle nh;
    ros::NodeHandle pnh(uniqueNs("can_driver_hw_smoke_csp_startup_limits"));

    pnh.setParam("joints", makeSingleCspJoint());
    setPositionLimits(pnh, "test_arm", -1.0, 1.0);

    ASSERT_TRUE(hw.init(nh, pnh));

    auto &coordinator = hw.operationalCoordinator();
    const auto initResult = coordinator.RequestInit("fake0", false);
    EXPECT_FALSE(initResult.ok);
    EXPECT_EQ(coordinator.mode(), can_driver::SystemOpMode::Configured);
    EXPECT_EQ(fakeDm->protocol()->setModeCalls(), 0);
}

TEST_F(CanDriverHWSmokeTest, CspInitAcceptsStartupPositionOnConfiguredUpperLimit)
{
    auto fakeDm = std::make_shared<FakeDeviceManager>();
    fakeDm->protocol()->setFeedbackPosition(rawFromPprRadians(1.0));

    CanDriverHW hw(fakeDm);

    ros::NodeHandle nh;
    ros::NodeHandle pnh(uniqueNs("can_driver_hw_smoke_csp_startup_limit_boundary"));

    pnh.setParam("joints", makeSingleCspJoint());
    setPositionLimits(pnh, "test_arm", -1.0, 1.0);

    ASSERT_TRUE(hw.init(nh, pnh));

    auto &coordinator = hw.operationalCoordinator();
    const auto initResult = coordinator.RequestInit("fake0", false);
    EXPECT_TRUE(initResult.ok) << initResult.message;
    EXPECT_EQ(coordinator.mode(), can_driver::SystemOpMode::Standby);
    EXPECT_EQ(fakeDm->protocol()->setModeCalls(), 1);
}

TEST_F(CanDriverHWSmokeTest, CspInitAcceptsStartupPositionWithinConfiguredTolerance)
{
    auto fakeDm = std::make_shared<FakeDeviceManager>();
    fakeDm->protocol()->setFeedbackPosition(rawFromPprRadians(1.00005));

    CanDriverHW hw(fakeDm);

    ros::NodeHandle nh;
    ros::NodeHandle pnh(uniqueNs("can_driver_hw_smoke_csp_startup_limit_tolerance"));

    pnh.setParam("joints", makeSingleCspJoint());
    setPositionLimits(pnh, "test_arm", -1.0, 1.0);
    pnh.setParam("startup_position_limit_tolerance_rad", 1e-4);

    ASSERT_TRUE(hw.init(nh, pnh));

    auto &coordinator = hw.operationalCoordinator();
    const auto initResult = coordinator.RequestInit("fake0", false);
    EXPECT_TRUE(initResult.ok) << initResult.message;
    EXPECT_EQ(coordinator.mode(), can_driver::SystemOpMode::Standby);
    EXPECT_EQ(fakeDm->protocol()->setModeCalls(), 1);
}

TEST_F(CanDriverHWSmokeTest, CspInitRejectsStartupPositionOutsideConfiguredTolerance)
{
    auto fakeDm = std::make_shared<FakeDeviceManager>();
    fakeDm->protocol()->setFeedbackPosition(rawFromPprRadians(1.0002));

    CanDriverHW hw(fakeDm);

    ros::NodeHandle nh;
    ros::NodeHandle pnh(uniqueNs("can_driver_hw_smoke_csp_startup_limit_tolerance_reject"));

    pnh.setParam("joints", makeSingleCspJoint());
    setPositionLimits(pnh, "test_arm", -1.0, 1.0);
    pnh.setParam("startup_position_limit_tolerance_rad", 1e-4);

    ASSERT_TRUE(hw.init(nh, pnh));

    auto &coordinator = hw.operationalCoordinator();
    const auto initResult = coordinator.RequestInit("fake0", false);
    EXPECT_FALSE(initResult.ok);
    EXPECT_EQ(coordinator.mode(), can_driver::SystemOpMode::Configured);
    EXPECT_EQ(fakeDm->protocol()->setModeCalls(), 0);
}

TEST_F(CanDriverHWSmokeTest, RunningCspJointUsesQuickSetPositionWithPprScale)
{
    auto fakeDm = std::make_shared<FakeDeviceManager>();
    CanDriverHW hw(fakeDm);

    ros::NodeHandle nh;
    ros::NodeHandle pnh(uniqueNs("can_driver_hw_smoke_csp_write"));

    pnh.setParam("joints", makeSingleCspJoint());
    pnh.setParam("debug_bypass_ros_control", true);
    pnh.setParam("motor_state_period_sec", 0.05);
    pnh.setParam("safety_require_enabled_for_motion", false);

    ASSERT_TRUE(hw.init(nh, pnh));
    enterRunning(hw);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    const std::string cmdTopic = pnh.resolveName("motor/test_arm/cmd_position");
    ros::Publisher pub = nh.advertise<std_msgs::Float64>(cmdTopic, 1);

    for (int i = 0; i < 20 && pub.getNumSubscribers() == 0; ++i) {
        ros::Duration(0.01).sleep();
    }

    std_msgs::Float64 msg;
    msg.data = 1.0;
    pub.publish(msg);
    ros::Duration(0.05).sleep();

    hw.write(ros::Time::now(), ros::Duration(0.01));

    const int32_t expectedRaw =
        static_cast<int32_t>(std::llround(1.0 / (2.0 * M_PI / 65536.0)));
    EXPECT_EQ(fakeDm->protocol()->quickPositionCalls(), 1);
    EXPECT_EQ(fakeDm->protocol()->positionCalls(), 0);
    EXPECT_EQ(fakeDm->protocol()->lastQuickPositionMotor(), 0x05u);
    EXPECT_EQ(fakeDm->protocol()->lastQuickPosition(), expectedRaw);

    spinner.stop();
}

TEST_F(CanDriverHWSmokeTest, RunningCspJointClampsCommandToConfiguredPositionLimits)
{
    auto fakeDm = std::make_shared<FakeDeviceManager>();
    CanDriverHW hw(fakeDm);

    ros::NodeHandle nh;
    ros::NodeHandle pnh(uniqueNs("can_driver_hw_smoke_csp_position_limits"));

    pnh.setParam("joints", makeSingleCspJoint());
    pnh.setParam("debug_bypass_ros_control", true);
    pnh.setParam("motor_state_period_sec", 0.05);
    pnh.setParam("safety_require_enabled_for_motion", false);
    setPositionLimits(pnh, "test_arm", -0.5, 0.5);

    ASSERT_TRUE(hw.init(nh, pnh));
    enterRunning(hw);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    const std::string cmdTopic = pnh.resolveName("motor/test_arm/cmd_position");
    ros::Publisher pub = nh.advertise<std_msgs::Float64>(cmdTopic, 1);

    for (int i = 0; i < 20 && pub.getNumSubscribers() == 0; ++i) {
        ros::Duration(0.01).sleep();
    }

    std_msgs::Float64 msg;
    msg.data = 1.0;
    pub.publish(msg);
    ros::Duration(0.05).sleep();

    hw.write(ros::Time::now(), ros::Duration(0.01));

    EXPECT_EQ(fakeDm->protocol()->quickPositionCalls(), 1);
    EXPECT_EQ(fakeDm->protocol()->lastQuickPositionMotor(), 0x05u);
    EXPECT_EQ(fakeDm->protocol()->lastQuickPosition(), rawFromPprRadians(0.5));

    spinner.stop();
}

TEST_F(CanDriverHWSmokeTest, RunningCspJointAppliesMaxPositionStepLimit)
{
    auto fakeDm = std::make_shared<FakeDeviceManager>();
    CanDriverHW hw(fakeDm);

    ros::NodeHandle nh;
    ros::NodeHandle pnh(uniqueNs("can_driver_hw_smoke_csp_step_limit"));

    pnh.setParam("joints", makeSingleCspJoint());
    pnh.setParam("debug_bypass_ros_control", true);
    pnh.setParam("motor_state_period_sec", 0.05);
    pnh.setParam("safety_require_enabled_for_motion", false);
    pnh.setParam("max_position_step_rad", 0.2);

    ASSERT_TRUE(hw.init(nh, pnh));
    enterRunning(hw);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    const std::string cmdTopic = pnh.resolveName("motor/test_arm/cmd_position");
    ros::Publisher pub = nh.advertise<std_msgs::Float64>(cmdTopic, 1);

    for (int i = 0; i < 20 && pub.getNumSubscribers() == 0; ++i) {
        ros::Duration(0.01).sleep();
    }

    std_msgs::Float64 msg;
    msg.data = 1.0;
    pub.publish(msg);
    ros::Duration(0.05).sleep();

    hw.write(ros::Time::now(), ros::Duration(0.01));

    EXPECT_EQ(fakeDm->protocol()->quickPositionCalls(), 1);
    EXPECT_EQ(fakeDm->protocol()->lastQuickPositionMotor(), 0x05u);
    EXPECT_EQ(fakeDm->protocol()->lastQuickPosition(), rawFromPprRadians(0.2));

    spinner.stop();
}

TEST_F(CanDriverHWSmokeTest, DispatchPreparedCommandsKeepsPrepareTimeRoutingSnapshot)
{
    auto fakeDm = std::make_shared<FakeDeviceManager>();

    std::deque<can_driver::CanDriverJointConfig> joints(1);
    joints[0].name = "test_arm";
    joints[0].motorId = static_cast<MotorID>(0x05);
    joints[0].protocol = CanType::PP;
    joints[0].canDevice = "fake0";
    joints[0].controlMode = "csp";
    joints[0].positionScale = 2.0 * M_PI / 65536.0;
    joints[0].velocityScale = 2.0 * M_PI / 65536.0;
    joints[0].pos = 0.0;
    joints[0].posCmd = 1.0;

    std::vector<can_driver::CanDriverDeviceProtocolGroup> groups{
        {"fake0", CanType::PP, {0}},
    };
    std::vector<int32_t> rawCommandBuffer(1, 0);
    std::vector<uint8_t> commandValidBuffer(1, 0);
    std::vector<can_driver::CanDriverPreparedCommand> preparedCommandBuffer(
        1, can_driver::CanDriverPreparedCommand{});
    std::mutex jointStateMutex;
    CommandGate commandGate;
    can_driver::CanDriverIoRuntime::WriteConfig config;
    config.safetyRequireEnabledForMotion = false;

    can_driver::CanDriverIoRuntime::PrepareCommands(&joints,
                                                    &rawCommandBuffer,
                                                    &commandValidBuffer,
                                                    &preparedCommandBuffer,
                                                    &jointStateMutex,
                                                    config);

    {
        std::lock_guard<std::mutex> lock(jointStateMutex);
        joints[0].controlMode = "velocity";
    }

    bool anyFaultObserved = false;
    can_driver::CanDriverIoRuntime::DispatchPreparedCommands(*fakeDm,
                                                             groups,
                                                             &joints,
                                                             rawCommandBuffer,
                                                             &commandValidBuffer,
                                                             preparedCommandBuffer,
                                                             &jointStateMutex,
                                                             &commandGate,
                                                             config,
                                                             &anyFaultObserved);

    EXPECT_FALSE(anyFaultObserved);
    EXPECT_EQ(fakeDm->protocol()->quickPositionCalls(), 1);
    EXPECT_EQ(fakeDm->protocol()->velocityCalls(), 0);
    EXPECT_EQ(fakeDm->protocol()->lastQuickPositionMotor(), 0x05u);
    EXPECT_EQ(fakeDm->protocol()->lastQuickPosition(), rawFromPprRadians(1.0));
}

TEST_F(CanDriverHWSmokeTest, SyncJointFeedbackAppliesNegativeDirectionSign)
{
    auto fakeDm = std::make_shared<FakeDeviceManager>();

    std::deque<can_driver::CanDriverJointConfig> joints(1);
    joints[0].name = "test_arm";
    joints[0].motorId = static_cast<MotorID>(0x05);
    joints[0].protocol = CanType::PP;
    joints[0].canDevice = "fake0";
    joints[0].controlMode = "csp";
    joints[0].positionScale = 2.0 * M_PI / 65536.0;
    joints[0].velocityScale = 2.0 * M_PI / 65536.0;
    joints[0].directionSign = -1.0;

    fakeDm->protocol()->setFeedbackPosition(rawFromPprRadians(0.25));
    fakeDm->protocol()->setFeedbackVelocity(static_cast<int16_t>(rawFromPprRadians(0.5)));
    fakeDm->protocol()->setFeedbackCurrent(7);

    std::vector<can_driver::CanDriverDeviceProtocolGroup> groups{
        {"fake0", CanType::PP, {0}},
    };
    std::mutex jointStateMutex;

    can_driver::CanDriverIoRuntime::SyncJointFeedback(*fakeDm,
                                                      groups,
                                                      &joints,
                                                      &jointStateMutex);

    std::lock_guard<std::mutex> lock(jointStateMutex);
    EXPECT_NEAR(joints[0].pos, -0.25, 1e-4);
    EXPECT_NEAR(joints[0].vel, -0.5, 1e-4);
    EXPECT_DOUBLE_EQ(joints[0].eff, 7.0);
}

TEST_F(CanDriverHWSmokeTest, PrepareCommandsAppliesNegativeDirectionSignToPosition)
{
    ros::Time::init();

    std::deque<can_driver::CanDriverJointConfig> joints(1);
    joints[0].name = "test_arm";
    joints[0].motorId = static_cast<MotorID>(0x05);
    joints[0].protocol = CanType::PP;
    joints[0].canDevice = "fake0";
    joints[0].controlMode = "csp";
    joints[0].positionScale = 2.0 * M_PI / 65536.0;
    joints[0].velocityScale = 2.0 * M_PI / 65536.0;
    joints[0].directionSign = -1.0;
    joints[0].posCmd = 0.25;

    std::vector<int32_t> rawCommandBuffer(1, 0);
    std::vector<uint8_t> commandValidBuffer(1, 0);
    std::vector<can_driver::CanDriverPreparedCommand> preparedCommandBuffer(
        1, can_driver::CanDriverPreparedCommand{});
    std::mutex jointStateMutex;
    can_driver::CanDriverIoRuntime::WriteConfig config;

    can_driver::CanDriverIoRuntime::PrepareCommands(&joints,
                                                    &rawCommandBuffer,
                                                    &commandValidBuffer,
                                                    &preparedCommandBuffer,
                                                    &jointStateMutex,
                                                    config);

    EXPECT_EQ(commandValidBuffer[0], 1);
    EXPECT_TRUE(preparedCommandBuffer[0].valid);
    EXPECT_EQ(rawCommandBuffer[0], -rawFromPprRadians(0.25));
}

TEST_F(CanDriverHWSmokeTest, PrepareCommandsSubtractsLocalZeroOffsetForPosition)
{
    ros::Time::init();

    std::deque<can_driver::CanDriverJointConfig> joints(1);
    joints[0].name = "test_arm";
    joints[0].motorId = static_cast<MotorID>(0x05);
    joints[0].protocol = CanType::PP;
    joints[0].canDevice = "fake0";
    joints[0].controlMode = "csp";
    joints[0].positionScale = 2.0 * M_PI / 65536.0;
    joints[0].velocityScale = 2.0 * M_PI / 65536.0;
    joints[0].zeroOffsetRad = -0.75;
    joints[0].posCmd = 0.25;

    std::vector<int32_t> rawCommandBuffer(1, 0);
    std::vector<uint8_t> commandValidBuffer(1, 0);
    std::vector<can_driver::CanDriverPreparedCommand> preparedCommandBuffer(
        1, can_driver::CanDriverPreparedCommand{});
    std::mutex jointStateMutex;
    can_driver::CanDriverIoRuntime::WriteConfig config;

    can_driver::CanDriverIoRuntime::PrepareCommands(&joints,
                                                    &rawCommandBuffer,
                                                    &commandValidBuffer,
                                                    &preparedCommandBuffer,
                                                    &jointStateMutex,
                                                    config);

    EXPECT_EQ(commandValidBuffer[0], 1);
    EXPECT_TRUE(preparedCommandBuffer[0].valid);
    EXPECT_EQ(rawCommandBuffer[0], rawFromPprRadians(1.0));
}

TEST_F(CanDriverHWSmokeTest, PrepareCommandsAppliesNegativeDirectionSignToVelocity)
{
    ros::Time::init();

    std::deque<can_driver::CanDriverJointConfig> joints(1);
    joints[0].name = "test_wheel";
    joints[0].motorId = static_cast<MotorID>(0x141);
    joints[0].protocol = CanType::MT;
    joints[0].canDevice = "fake0";
    joints[0].controlMode = "velocity";
    joints[0].positionScale = 1.0;
    joints[0].velocityScale = 0.1;
    joints[0].directionSign = -1.0;
    joints[0].velCmd = 1.2;

    std::vector<int32_t> rawCommandBuffer(1, 0);
    std::vector<uint8_t> commandValidBuffer(1, 0);
    std::vector<can_driver::CanDriverPreparedCommand> preparedCommandBuffer(
        1, can_driver::CanDriverPreparedCommand{});
    std::mutex jointStateMutex;
    can_driver::CanDriverIoRuntime::WriteConfig config;

    can_driver::CanDriverIoRuntime::PrepareCommands(&joints,
                                                    &rawCommandBuffer,
                                                    &commandValidBuffer,
                                                    &preparedCommandBuffer,
                                                    &jointStateMutex,
                                                    config);

    EXPECT_EQ(commandValidBuffer[0], 1);
    EXPECT_TRUE(preparedCommandBuffer[0].valid);
    EXPECT_EQ(rawCommandBuffer[0], -12);
}

TEST_F(CanDriverHWSmokeTest, SetZeroLimitServiceAcceptsCspJoint)
{
    auto fakeDm = std::make_shared<FakeDeviceManager>();
    fakeDm->protocol()->setFeedbackPosition(rawFromPprRadians(0.1));
    CanDriverHW hw(fakeDm);

    ros::NodeHandle nh;
    ros::NodeHandle pnh(uniqueNs("can_driver_hw_smoke_csp_zero_limit"));

    pnh.setParam("joints", makeSingleCspJoint());

    ASSERT_TRUE(hw.init(nh, pnh));
    const auto initResult = hw.operationalCoordinator().RequestInit("fake0", false);
    ASSERT_TRUE(initResult.ok) << initResult.message;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::ServiceClient client = nh.serviceClient<can_driver::SetZeroLimit>(
        pnh.resolveName("set_zero_limit"));
    ASSERT_TRUE(client.waitForExistence(ros::Duration(1.0)));

    can_driver::SetZeroLimit srv;
    srv.request.motor_id = 0x05u;
    srv.request.zero_offset_rad = 0.0;
    srv.request.use_current_position_as_zero = false;
    srv.request.min_position_rad = -0.4;
    srv.request.max_position_rad = 0.4;
    srv.request.use_urdf_limits = false;
    srv.request.apply_to_motor = false;
    ASSERT_TRUE(client.call(srv));
    ASSERT_TRUE(srv.response.success) << srv.response.message;
    EXPECT_DOUBLE_EQ(srv.response.applied_zero_offset_rad, 0.0);
    EXPECT_NEAR(srv.response.current_position_rad, 0.1, 1e-4);
    EXPECT_DOUBLE_EQ(srv.response.applied_min_rad, -0.4);
    EXPECT_DOUBLE_EQ(srv.response.applied_max_rad, 0.4);

    spinner.stop();
}

TEST_F(CanDriverHWSmokeTest, SetZeroLimitServiceCanAutoZeroFromCurrentPosition)
{
    auto fakeDm = std::make_shared<FakeDeviceManager>();
    fakeDm->protocol()->setFeedbackPosition(rawFromPprRadians(0.1));
    CanDriverHW hw(fakeDm);

    ros::NodeHandle nh;
    ros::NodeHandle pnh(uniqueNs("can_driver_hw_smoke_csp_auto_zero"));

    pnh.setParam("joints", makeSingleCspJoint());

    ASSERT_TRUE(hw.init(nh, pnh));
    const auto initResult = hw.operationalCoordinator().RequestInit("fake0", false);
    ASSERT_TRUE(initResult.ok) << initResult.message;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::ServiceClient client = nh.serviceClient<can_driver::SetZeroLimit>(
        pnh.resolveName("set_zero_limit"));
    ASSERT_TRUE(client.waitForExistence(ros::Duration(1.0)));

    setFreshEnabledFeedback(*fakeDm, "fake0", CanType::PP, static_cast<MotorID>(0x05), false);

    can_driver::SetZeroLimit srv;
    srv.request.motor_id = 0x05u;
    srv.request.zero_offset_rad = 999.0;
    srv.request.use_current_position_as_zero = true;
    srv.request.min_position_rad = -0.4;
    srv.request.max_position_rad = 0.4;
    srv.request.use_urdf_limits = false;
    srv.request.apply_to_motor = true;
    ASSERT_TRUE(client.call(srv));
    ASSERT_TRUE(srv.response.success) << srv.response.message;
    EXPECT_NEAR(srv.response.current_position_rad, 0.1, 1e-4);
    EXPECT_NEAR(srv.response.applied_zero_offset_rad, -0.1, 1e-4);
    EXPECT_NEAR(srv.response.applied_min_rad, -0.4, 1e-4);
    EXPECT_NEAR(srv.response.applied_max_rad, 0.4, 1e-4);
    EXPECT_EQ(fakeDm->protocol()->setOffsetCalls(), 1);
    EXPECT_EQ(fakeDm->protocol()->persistCalls(), 1);
    EXPECT_EQ(fakeDm->protocol()->setLimitCalls(), 1);
    EXPECT_EQ(fakeDm->protocol()->lastOffsetRaw(), -rawFromPprRadians(0.1));
    EXPECT_EQ(fakeDm->protocol()->lastLimitMin(), rawFromPprRadians(-0.4));
    EXPECT_EQ(fakeDm->protocol()->lastLimitMax(), rawFromPprRadians(0.4));
    EXPECT_TRUE(fakeDm->protocol()->lastLimitEnable());

    spinner.stop();
}

TEST_F(CanDriverHWSmokeTest, SetZeroLimitServiceRejectsHardwareWriteWhileEnabled)
{
    auto fakeDm = std::make_shared<FakeDeviceManager>();
    fakeDm->protocol()->setFeedbackPosition(rawFromPprRadians(0.1));
    CanDriverHW hw(fakeDm);

    ros::NodeHandle nh;
    ros::NodeHandle pnh(uniqueNs("can_driver_hw_smoke_zero_limit_enabled_reject"));

    pnh.setParam("joints", makeSingleCspJoint());

    ASSERT_TRUE(hw.init(nh, pnh));
    const auto initResult = hw.operationalCoordinator().RequestInit("fake0", false);
    ASSERT_TRUE(initResult.ok) << initResult.message;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::ServiceClient client = nh.serviceClient<can_driver::SetZeroLimit>(
        pnh.resolveName("set_zero_limit"));
    ASSERT_TRUE(client.waitForExistence(ros::Duration(1.0)));

    setFreshEnabledFeedback(*fakeDm, "fake0", CanType::PP, static_cast<MotorID>(0x05), true);

    can_driver::SetZeroLimit srv;
    srv.request.motor_id = 0x05u;
    srv.request.zero_offset_rad = 0.0;
    srv.request.use_current_position_as_zero = false;
    srv.request.min_position_rad = -0.4;
    srv.request.max_position_rad = 0.4;
    srv.request.use_urdf_limits = false;
    srv.request.apply_to_motor = true;
    ASSERT_TRUE(client.call(srv));
    EXPECT_FALSE(srv.response.success);
    EXPECT_NE(srv.response.message.find("disabled first"), std::string::npos);

    spinner.stop();
}

TEST_F(CanDriverHWSmokeTest, SetZeroServiceAllowsAutoZeroOutsideCurrentLimits)
{
    auto fakeDm = std::make_shared<FakeDeviceManager>();
    fakeDm->protocol()->setFeedbackPosition(rawFromPprRadians(1.2));
    CanDriverHW hw(fakeDm);

    ros::NodeHandle nh;
    ros::NodeHandle pnh(uniqueNs("can_driver_hw_smoke_zero_only_outside_limits"));

    pnh.setParam("joints", makeSingleCspJoint());

    ASSERT_TRUE(hw.init(nh, pnh));
    const auto initResult = hw.operationalCoordinator().RequestInit("fake0", false);
    ASSERT_TRUE(initResult.ok) << initResult.message;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::ServiceClient client = nh.serviceClient<can_driver::SetZero>(
        pnh.resolveName("set_zero"));
    ASSERT_TRUE(client.waitForExistence(ros::Duration(1.0)));

    setFreshEnabledFeedback(*fakeDm, "fake0", CanType::PP, static_cast<MotorID>(0x05), false);

    can_driver::SetZero srv;
    srv.request.motor_id = 0x05u;
    srv.request.zero_offset_rad = 0.0;
    srv.request.use_current_position_as_zero = true;
    srv.request.apply_to_motor = true;
    ASSERT_TRUE(client.call(srv));
    ASSERT_TRUE(srv.response.success) << srv.response.message;
    EXPECT_NEAR(srv.response.current_position_rad, 1.2, 1e-4);
    EXPECT_NEAR(srv.response.applied_zero_offset_rad, -1.2, 1e-4);
    EXPECT_EQ(fakeDm->protocol()->setOffsetCalls(), 1);
    EXPECT_EQ(fakeDm->protocol()->persistCalls(), 1);

    spinner.stop();
}

TEST_F(CanDriverHWSmokeTest, SetZeroServiceApplyToMotorCanSkipPersistence)
{
    auto fakeDm = std::make_shared<FakeDeviceManager>();
    fakeDm->protocol()->setFeedbackPosition(rawFromPprRadians(0.1));
    fakeDm->protocol()->setPersistResult(false);
    CanDriverHW hw(fakeDm);

    ros::NodeHandle nh;
    ros::NodeHandle pnh(uniqueNs("can_driver_hw_smoke_zero_apply_without_persist"));

    pnh.setParam("joints", makeSingleCspJoint());
    pnh.setParam("pp_zero_offset_persist_on_apply_to_motor", false);

    ASSERT_TRUE(hw.init(nh, pnh));
    const auto initResult = hw.operationalCoordinator().RequestInit("fake0", false);
    ASSERT_TRUE(initResult.ok) << initResult.message;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::ServiceClient client = nh.serviceClient<can_driver::SetZero>(
        pnh.resolveName("set_zero"));
    ASSERT_TRUE(client.waitForExistence(ros::Duration(1.0)));

    setFreshEnabledFeedback(*fakeDm, "fake0", CanType::PP, static_cast<MotorID>(0x05), false);

    can_driver::SetZero srv;
    srv.request.motor_id = 0x05u;
    srv.request.zero_offset_rad = 0.0;
    srv.request.use_current_position_as_zero = true;
    srv.request.apply_to_motor = true;
    ASSERT_TRUE(client.call(srv));
    ASSERT_TRUE(srv.response.success) << srv.response.message;
    EXPECT_NEAR(srv.response.applied_zero_offset_rad, -0.1, 1e-4);
    EXPECT_EQ(fakeDm->protocol()->setOffsetCalls(), 1);
    EXPECT_EQ(fakeDm->protocol()->persistCalls(), 0);
    EXPECT_EQ(fakeDm->protocol()->lastOffsetRaw(), -rawFromPprRadians(0.1));

    spinner.stop();
}

TEST_F(CanDriverHWSmokeTest, SetZeroServiceAppliesDirectionSignToZeroOffsets)
{
    auto fakeDm = std::make_shared<FakeDeviceManager>();
    fakeDm->protocol()->setFeedbackPosition(rawFromPprRadians(0.1));
    ASSERT_TRUE(fakeDm->protocol()->setPositionOffset(static_cast<MotorID>(0x05),
                                                      rawFromPprRadians(0.05)));
    CanDriverHW hw(fakeDm);

    ros::NodeHandle nh;
    ros::NodeHandle pnh(uniqueNs("can_driver_hw_smoke_zero_negative_direction"));

    pnh.setParam("joints", makeSingleCspJoint(-1.0));

    ASSERT_TRUE(hw.init(nh, pnh));
    const auto initResult = hw.operationalCoordinator().RequestInit("fake0", false);
    ASSERT_TRUE(initResult.ok) << initResult.message;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::ServiceClient client = nh.serviceClient<can_driver::SetZero>(
        pnh.resolveName("set_zero"));
    ASSERT_TRUE(client.waitForExistence(ros::Duration(1.0)));

    setFreshEnabledFeedback(*fakeDm, "fake0", CanType::PP, static_cast<MotorID>(0x05), false);

    can_driver::SetZero srv;
    srv.request.motor_id = 0x05u;
    srv.request.zero_offset_rad = 0.0;
    srv.request.use_current_position_as_zero = true;
    srv.request.apply_to_motor = true;
    ASSERT_TRUE(client.call(srv));
    ASSERT_TRUE(srv.response.success) << srv.response.message;
    EXPECT_NEAR(srv.response.current_position_rad, -0.1, 1e-4);
    EXPECT_NEAR(srv.response.previous_zero_offset_rad, -0.05, 1e-4);
    EXPECT_NEAR(srv.response.applied_zero_offset_rad, 0.05, 1e-4);
    EXPECT_EQ(fakeDm->protocol()->persistCalls(), 1);
    const double signedPprScale = -(2.0 * M_PI / 65536.0);
    EXPECT_EQ(fakeDm->protocol()->lastOffsetRaw(),
              static_cast<int32_t>(std::llround(
                  srv.response.applied_zero_offset_rad / signedPprScale)));

    spinner.stop();
}

TEST_F(CanDriverHWSmokeTest, SetZeroServiceRollsBackOffsetWhenPersistenceFails)
{
    auto fakeDm = std::make_shared<FakeDeviceManager>();
    fakeDm->protocol()->setFeedbackPosition(rawFromPprRadians(0.1));
    ASSERT_TRUE(fakeDm->protocol()->setPositionOffset(static_cast<MotorID>(0x05),
                                                      rawFromPprRadians(0.05)));
    fakeDm->protocol()->setPersistResult(false);
    CanDriverHW hw(fakeDm);

    ros::NodeHandle nh;
    ros::NodeHandle pnh(uniqueNs("can_driver_hw_smoke_zero_persist_fail"));

    pnh.setParam("joints", makeSingleCspJoint());

    ASSERT_TRUE(hw.init(nh, pnh));
    const auto initResult = hw.operationalCoordinator().RequestInit("fake0", false);
    ASSERT_TRUE(initResult.ok) << initResult.message;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::ServiceClient client = nh.serviceClient<can_driver::SetZero>(
        pnh.resolveName("set_zero"));
    ASSERT_TRUE(client.waitForExistence(ros::Duration(1.0)));

    setFreshEnabledFeedback(*fakeDm, "fake0", CanType::PP, static_cast<MotorID>(0x05), false);

    can_driver::SetZero srv;
    srv.request.motor_id = 0x05u;
    srv.request.zero_offset_rad = 0.0;
    srv.request.use_current_position_as_zero = true;
    srv.request.apply_to_motor = true;
    ASSERT_TRUE(client.call(srv));
    EXPECT_FALSE(srv.response.success);
    EXPECT_NE(srv.response.message.find("Rollback"), std::string::npos);
    EXPECT_EQ(fakeDm->protocol()->setOffsetCalls(), 3);
    EXPECT_EQ(fakeDm->protocol()->persistCalls(), 2);
    EXPECT_EQ(fakeDm->protocol()->lastOffsetRaw(), rawFromPprRadians(0.05));

    spinner.stop();
}

TEST_F(CanDriverHWSmokeTest, LocalPersistedZeroOffsetRestoresOnNextInit)
{
    const std::string persistFile = uniqueTempFile("can_driver_zero_offsets");
    std::filesystem::remove(persistFile);

    {
        auto fakeDm = std::make_shared<FakeDeviceManager>();
        CanDriverHW hw(fakeDm);

        ros::NodeHandle nh;
        ros::NodeHandle pnh(uniqueNs("can_driver_hw_smoke_zero_local_persist_write"));

        pnh.setParam("joints", makeSingleCspJoint());
        pnh.setParam("pp_local_zero_offset_persistence_enabled", true);
        pnh.setParam("pp_local_zero_offset_file", persistFile);

        ASSERT_TRUE(hw.init(nh, pnh));
        const auto initResult = hw.operationalCoordinator().RequestInit("fake0", false);
        ASSERT_TRUE(initResult.ok) << initResult.message;

        ros::AsyncSpinner spinner(1);
        spinner.start();

        ros::ServiceClient client = nh.serviceClient<can_driver::SetZero>(
            pnh.resolveName("set_zero"));
        ASSERT_TRUE(client.waitForExistence(ros::Duration(1.0)));

        can_driver::SetZero srv;
        srv.request.motor_id = 0x05u;
        srv.request.zero_offset_rad = 0.2;
        srv.request.use_current_position_as_zero = false;
        srv.request.apply_to_motor = false;
        ASSERT_TRUE(client.call(srv));
        ASSERT_TRUE(srv.response.success) << srv.response.message;

        spinner.stop();
    }

    {
        std::ifstream in(persistFile);
        ASSERT_TRUE(in.is_open());
        std::string line;
        ASSERT_TRUE(static_cast<bool>(std::getline(in, line)));
        EXPECT_NE(line.find("5 0.20000000000000001"), std::string::npos);
    }

    {
        auto fakeDm = std::make_shared<FakeDeviceManager>();
        CanDriverHW hw(fakeDm);

        ros::NodeHandle nh;
        ros::NodeHandle pnh(uniqueNs("can_driver_hw_smoke_zero_local_persist_restore"));

        pnh.setParam("joints", makeSingleCspJoint());
        pnh.setParam("pp_local_zero_offset_persistence_enabled", true);
        pnh.setParam("pp_local_zero_offset_file", persistFile);

        ASSERT_TRUE(hw.init(nh, pnh));
        const auto initResult = hw.operationalCoordinator().RequestInit("fake0", false);
        ASSERT_TRUE(initResult.ok) << initResult.message;

        EXPECT_EQ(fakeDm->protocol()->setOffsetCalls(), 1);
        EXPECT_EQ(fakeDm->protocol()->lastOffsetRaw(), rawFromPprRadians(0.2));
    }

    std::filesystem::remove(persistFile);
}

TEST_F(CanDriverHWSmokeTest, SetZeroLimitServiceStopsBeforeLimitsWhenZeroPersistenceFails)
{
    auto fakeDm = std::make_shared<FakeDeviceManager>();
    fakeDm->protocol()->setFeedbackPosition(rawFromPprRadians(0.1));
    ASSERT_TRUE(fakeDm->protocol()->setPositionOffset(static_cast<MotorID>(0x05),
                                                      rawFromPprRadians(0.05)));
    fakeDm->protocol()->setPersistResult(false);
    CanDriverHW hw(fakeDm);

    ros::NodeHandle nh;
    ros::NodeHandle pnh(uniqueNs("can_driver_hw_smoke_zero_limit_persist_fail"));

    pnh.setParam("joints", makeSingleCspJoint());

    ASSERT_TRUE(hw.init(nh, pnh));
    const auto initResult = hw.operationalCoordinator().RequestInit("fake0", false);
    ASSERT_TRUE(initResult.ok) << initResult.message;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::ServiceClient client = nh.serviceClient<can_driver::SetZeroLimit>(
        pnh.resolveName("set_zero_limit"));
    ASSERT_TRUE(client.waitForExistence(ros::Duration(1.0)));

    setFreshEnabledFeedback(*fakeDm, "fake0", CanType::PP, static_cast<MotorID>(0x05), false);

    can_driver::SetZeroLimit srv;
    srv.request.motor_id = 0x05u;
    srv.request.zero_offset_rad = 0.0;
    srv.request.use_current_position_as_zero = true;
    srv.request.min_position_rad = -0.4;
    srv.request.max_position_rad = 0.4;
    srv.request.use_urdf_limits = false;
    srv.request.apply_to_motor = true;
    ASSERT_TRUE(client.call(srv));
    EXPECT_FALSE(srv.response.success);
    EXPECT_EQ(fakeDm->protocol()->persistCalls(), 2);
    EXPECT_EQ(fakeDm->protocol()->setLimitCalls(), 0);

    spinner.stop();
}

TEST_F(CanDriverHWSmokeTest, SetZeroLimitServiceApplyToMotorCanSkipPersistence)
{
    auto fakeDm = std::make_shared<FakeDeviceManager>();
    fakeDm->protocol()->setFeedbackPosition(rawFromPprRadians(0.1));
    fakeDm->protocol()->setPersistResult(false);
    CanDriverHW hw(fakeDm);

    ros::NodeHandle nh;
    ros::NodeHandle pnh(uniqueNs("can_driver_hw_smoke_zero_limit_apply_without_persist"));

    pnh.setParam("joints", makeSingleCspJoint());
    pnh.setParam("pp_zero_offset_persist_on_apply_to_motor", false);

    ASSERT_TRUE(hw.init(nh, pnh));
    const auto initResult = hw.operationalCoordinator().RequestInit("fake0", false);
    ASSERT_TRUE(initResult.ok) << initResult.message;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::ServiceClient client = nh.serviceClient<can_driver::SetZeroLimit>(
        pnh.resolveName("set_zero_limit"));
    ASSERT_TRUE(client.waitForExistence(ros::Duration(1.0)));

    setFreshEnabledFeedback(*fakeDm, "fake0", CanType::PP, static_cast<MotorID>(0x05), false);

    can_driver::SetZeroLimit srv;
    srv.request.motor_id = 0x05u;
    srv.request.zero_offset_rad = 0.0;
    srv.request.use_current_position_as_zero = true;
    srv.request.min_position_rad = -0.4;
    srv.request.max_position_rad = 0.4;
    srv.request.use_urdf_limits = false;
    srv.request.apply_to_motor = true;
    ASSERT_TRUE(client.call(srv));
    ASSERT_TRUE(srv.response.success) << srv.response.message;
    EXPECT_EQ(fakeDm->protocol()->setOffsetCalls(), 1);
    EXPECT_EQ(fakeDm->protocol()->persistCalls(), 0);
    EXPECT_EQ(fakeDm->protocol()->setLimitCalls(), 1);
    EXPECT_EQ(fakeDm->protocol()->lastOffsetRaw(), -rawFromPprRadians(0.1));
    EXPECT_EQ(fakeDm->protocol()->lastLimitMin(), rawFromPprRadians(-0.4));
    EXPECT_EQ(fakeDm->protocol()->lastLimitMax(), rawFromPprRadians(0.4));
    EXPECT_TRUE(fakeDm->protocol()->lastLimitEnable());

    spinner.stop();
}

TEST_F(CanDriverHWSmokeTest, ApplyLimitsServiceRejectsOutsidePositionWhenRequired)
{
    auto fakeDm = std::make_shared<FakeDeviceManager>();
    fakeDm->protocol()->setFeedbackPosition(rawFromPprRadians(1.2));
    CanDriverHW hw(fakeDm);

    ros::NodeHandle nh;
    ros::NodeHandle pnh(uniqueNs("can_driver_hw_smoke_apply_limits_strict"));

    pnh.setParam("joints", makeSingleCspJoint());

    ASSERT_TRUE(hw.init(nh, pnh));
    const auto initResult = hw.operationalCoordinator().RequestInit("fake0", false);
    ASSERT_TRUE(initResult.ok) << initResult.message;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::ServiceClient client = nh.serviceClient<can_driver::ApplyLimits>(
        pnh.resolveName("apply_limits"));
    ASSERT_TRUE(client.waitForExistence(ros::Duration(1.0)));

    can_driver::ApplyLimits srv;
    srv.request.motor_id = 0x05u;
    srv.request.min_position_rad = -0.4;
    srv.request.max_position_rad = 0.4;
    srv.request.use_urdf_limits = false;
    srv.request.apply_to_motor = false;
    srv.request.require_current_inside_limits = true;
    ASSERT_TRUE(client.call(srv));
    EXPECT_FALSE(srv.response.success);
    EXPECT_NE(srv.response.message.find("outside requested limit range"), std::string::npos);

    spinner.stop();
}

TEST_F(CanDriverHWSmokeTest, ApplyLimitsServiceAllowsOutsidePositionWhenNotRequired)
{
    auto fakeDm = std::make_shared<FakeDeviceManager>();
    fakeDm->protocol()->setFeedbackPosition(rawFromPprRadians(1.2));
    CanDriverHW hw(fakeDm);

    ros::NodeHandle nh;
    ros::NodeHandle pnh(uniqueNs("can_driver_hw_smoke_apply_limits_relaxed"));

    pnh.setParam("joints", makeSingleCspJoint());

    ASSERT_TRUE(hw.init(nh, pnh));
    const auto initResult = hw.operationalCoordinator().RequestInit("fake0", false);
    ASSERT_TRUE(initResult.ok) << initResult.message;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::ServiceClient client = nh.serviceClient<can_driver::ApplyLimits>(
        pnh.resolveName("apply_limits"));
    ASSERT_TRUE(client.waitForExistence(ros::Duration(1.0)));

    can_driver::ApplyLimits srv;
    srv.request.motor_id = 0x05u;
    srv.request.min_position_rad = -0.4;
    srv.request.max_position_rad = 0.4;
    srv.request.use_urdf_limits = false;
    srv.request.apply_to_motor = false;
    srv.request.require_current_inside_limits = false;
    ASSERT_TRUE(client.call(srv));
    ASSERT_TRUE(srv.response.success) << srv.response.message;
    EXPECT_NEAR(srv.response.current_position_rad, 1.2, 1e-4);
    EXPECT_DOUBLE_EQ(srv.response.applied_min_rad, -0.4);
    EXPECT_DOUBLE_EQ(srv.response.applied_max_rad, 0.4);

    spinner.stop();
}

TEST_F(CanDriverHWSmokeTest, ApplyLimitsServiceOrdersRawLimitsForNegativeDirectionJoint)
{
    auto fakeDm = std::make_shared<FakeDeviceManager>();
    CanDriverHW hw(fakeDm);

    ros::NodeHandle nh;
    ros::NodeHandle pnh(uniqueNs("can_driver_hw_smoke_limits_negative_direction"));

    pnh.setParam("joints", makeSingleCspJoint(-1.0));

    ASSERT_TRUE(hw.init(nh, pnh));
    const auto initResult = hw.operationalCoordinator().RequestInit("fake0", false);
    ASSERT_TRUE(initResult.ok) << initResult.message;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::ServiceClient client = nh.serviceClient<can_driver::ApplyLimits>(
        pnh.resolveName("apply_limits"));
    ASSERT_TRUE(client.waitForExistence(ros::Duration(1.0)));

    setFreshEnabledFeedback(*fakeDm, "fake0", CanType::PP, static_cast<MotorID>(0x05), false);

    can_driver::ApplyLimits srv;
    srv.request.motor_id = 0x05u;
    srv.request.min_position_rad = -0.4;
    srv.request.max_position_rad = 0.2;
    srv.request.use_urdf_limits = false;
    srv.request.apply_to_motor = true;
    srv.request.require_current_inside_limits = false;
    ASSERT_TRUE(client.call(srv));
    ASSERT_TRUE(srv.response.success) << srv.response.message;
    EXPECT_DOUBLE_EQ(srv.response.applied_min_rad, -0.4);
    EXPECT_DOUBLE_EQ(srv.response.applied_max_rad, 0.2);
    EXPECT_EQ(fakeDm->protocol()->setLimitCalls(), 1);
    EXPECT_EQ(fakeDm->protocol()->lastLimitMin(), -rawFromPprRadians(0.2));
    EXPECT_EQ(fakeDm->protocol()->lastLimitMax(), rawFromPprRadians(0.4));

    spinner.stop();
}

TEST_F(CanDriverHWSmokeTest, SetModeServiceUpdatesRuntimeRoutingToVelocity)
{
    auto fakeDm = std::make_shared<FakeDeviceManager>();
    CanDriverHW hw(fakeDm);

    ros::NodeHandle nh;
    ros::NodeHandle pnh(uniqueNs("can_driver_hw_smoke_set_mode_routing"));

    pnh.setParam("joints", makeSingleCspJoint());
    pnh.setParam("debug_bypass_ros_control", true);
    pnh.setParam("motor_state_period_sec", 0.05);
    pnh.setParam("safety_require_enabled_for_motion", false);

    ASSERT_TRUE(hw.init(nh, pnh));
    enterRunning(hw);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::ServiceClient client = nh.serviceClient<can_driver::MotorCommand>(
        pnh.resolveName("motor_command"));
    ASSERT_TRUE(client.waitForExistence(ros::Duration(1.0)));

    ros::Publisher velPub = nh.advertise<std_msgs::Float64>(
        pnh.resolveName("motor/test_arm/cmd_velocity"), 1);
    for (int i = 0; i < 20 && velPub.getNumSubscribers() == 0; ++i) {
        ros::Duration(0.01).sleep();
    }

    can_driver::MotorCommand setModeSrv;
    setModeSrv.request.motor_id = 0x05u;
    setModeSrv.request.command = can_driver::MotorCommand::Request::CMD_SET_MODE;
    setModeSrv.request.value = 1.0;
    setFreshEnabledFeedback(*fakeDm, "fake0", CanType::PP, static_cast<MotorID>(0x05), false);
    ASSERT_TRUE(client.call(setModeSrv));
    ASSERT_TRUE(setModeSrv.response.success) << setModeSrv.response.message;

    setFreshEnabledFeedback(*fakeDm, "fake0", CanType::PP, static_cast<MotorID>(0x05), true);

    std_msgs::Float64 msg;
    msg.data = 0.0;
    velPub.publish(msg);
    ros::Duration(0.05).sleep();
    hw.write(ros::Time::now(), ros::Duration(0.01));
    EXPECT_EQ(fakeDm->protocol()->velocityCalls(), 2);
    EXPECT_EQ(fakeDm->protocol()->lastVelocityMotor(), 0x05u);
    EXPECT_EQ(fakeDm->protocol()->lastVelocity(), 0);

    msg.data = 1.25;
    velPub.publish(msg);
    ros::Duration(0.05).sleep();
    hw.write(ros::Time::now(), ros::Duration(0.01));

    EXPECT_EQ(fakeDm->protocol()->velocityCalls(), 3);
    EXPECT_EQ(fakeDm->protocol()->lastVelocityMotor(), 0x05u);
    EXPECT_EQ(fakeDm->protocol()->lastVelocity(), 13038);
    EXPECT_EQ(fakeDm->protocol()->quickPositionCalls(), 0);

    spinner.stop();
}

TEST_F(CanDriverHWSmokeTest, SetModeServicePreloadsCurrentPositionWhenSwitchingToPosition)
{
    auto fakeDm = std::make_shared<FakeDeviceManager>();
    fakeDm->protocol()->setFeedbackPosition(1024);
    CanDriverHW hw(fakeDm);

    ros::NodeHandle nh;
    ros::NodeHandle pnh(uniqueNs("can_driver_hw_smoke_set_mode_position_preload"));

    pnh.setParam("joints", makeSingleCspJoint());
    pnh.setParam("motor_state_period_sec", 0.05);

    ASSERT_TRUE(hw.init(nh, pnh));
    enterRunning(hw);
    hw.read(ros::Time::now(), ros::Duration(0.01));

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::ServiceClient client = nh.serviceClient<can_driver::MotorCommand>(
        pnh.resolveName("motor_command"));
    ASSERT_TRUE(client.waitForExistence(ros::Duration(1.0)));

    can_driver::MotorCommand setModeSrv;
    setModeSrv.request.motor_id = 0x05u;
    setModeSrv.request.command = can_driver::MotorCommand::Request::CMD_SET_MODE;
    setModeSrv.request.value = 0.0;
    setFreshEnabledFeedback(*fakeDm, "fake0", CanType::PP, static_cast<MotorID>(0x05), false);
    ASSERT_TRUE(client.call(setModeSrv));
    ASSERT_TRUE(setModeSrv.response.success) << setModeSrv.response.message;

    EXPECT_EQ(fakeDm->protocol()->positionCalls(), 1);
    EXPECT_EQ(fakeDm->protocol()->lastPositionMotor(), 0x05u);
    EXPECT_EQ(fakeDm->protocol()->lastPosition(), 1024);

    spinner.stop();
}

TEST_F(CanDriverHWSmokeTest, VelocityModeRequiresAlignedCommandOnceAfterModeSwitch)
{
    auto fakeDm = std::make_shared<FakeDeviceManager>();
    CanDriverHW hw(fakeDm);

    ros::NodeHandle nh;
    ros::NodeHandle pnh(uniqueNs("can_driver_hw_smoke_set_mode_velocity_alignment"));

    pnh.setParam("joints", makeSingleCspJoint());
    pnh.setParam("debug_bypass_ros_control", true);
    pnh.setParam("motor_state_period_sec", 0.05);
    pnh.setParam("safety_require_enabled_for_motion", false);

    ASSERT_TRUE(hw.init(nh, pnh));
    enterRunning(hw);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::ServiceClient client = nh.serviceClient<can_driver::MotorCommand>(
        pnh.resolveName("motor_command"));
    ASSERT_TRUE(client.waitForExistence(ros::Duration(1.0)));

    ros::Publisher velPub = nh.advertise<std_msgs::Float64>(
        pnh.resolveName("motor/test_arm/cmd_velocity"), 1);
    for (int i = 0; i < 20 && velPub.getNumSubscribers() == 0; ++i) {
        ros::Duration(0.01).sleep();
    }

    can_driver::MotorCommand setModeSrv;
    setModeSrv.request.motor_id = 0x05u;
    setModeSrv.request.command = can_driver::MotorCommand::Request::CMD_SET_MODE;
    setModeSrv.request.value = 1.0;
    setFreshEnabledFeedback(*fakeDm, "fake0", CanType::PP, static_cast<MotorID>(0x05), false);
    ASSERT_TRUE(client.call(setModeSrv));
    ASSERT_TRUE(setModeSrv.response.success) << setModeSrv.response.message;
    EXPECT_EQ(fakeDm->protocol()->velocityCalls(), 1);
    EXPECT_EQ(fakeDm->protocol()->lastVelocity(), 0);

    setFreshEnabledFeedback(*fakeDm, "fake0", CanType::PP, static_cast<MotorID>(0x05), true);

    std_msgs::Float64 msg;
    msg.data = 1.25;
    velPub.publish(msg);
    ros::Duration(0.05).sleep();
    hw.write(ros::Time::now(), ros::Duration(0.01));
    EXPECT_EQ(fakeDm->protocol()->velocityCalls(), 1);

    msg.data = 0.0;
    velPub.publish(msg);
    ros::Duration(0.05).sleep();
    hw.write(ros::Time::now(), ros::Duration(0.01));
    EXPECT_EQ(fakeDm->protocol()->velocityCalls(), 2);
    EXPECT_EQ(fakeDm->protocol()->lastVelocity(), 0);

    msg.data = 1.25;
    velPub.publish(msg);
    ros::Duration(0.05).sleep();
    hw.write(ros::Time::now(), ros::Duration(0.01));
    EXPECT_EQ(fakeDm->protocol()->velocityCalls(), 3);
    EXPECT_EQ(fakeDm->protocol()->lastVelocity(), 13038);

    spinner.stop();
}

TEST_F(CanDriverHWSmokeTest, SetModeServiceRejectsWhileEnabled)
{
    auto fakeDm = std::make_shared<FakeDeviceManager>();
    CanDriverHW hw(fakeDm);

    ros::NodeHandle nh;
    ros::NodeHandle pnh(uniqueNs("can_driver_hw_smoke_set_mode_enabled_reject"));

    pnh.setParam("joints", makeSingleCspJoint());
    pnh.setParam("motor_state_period_sec", 0.05);

    ASSERT_TRUE(hw.init(nh, pnh));
    enterRunning(hw);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::ServiceClient client = nh.serviceClient<can_driver::MotorCommand>(
        pnh.resolveName("motor_command"));
    ASSERT_TRUE(client.waitForExistence(ros::Duration(1.0)));

    setFreshEnabledFeedback(*fakeDm, "fake0", CanType::PP, static_cast<MotorID>(0x05), true);

    can_driver::MotorCommand setModeSrv;
    setModeSrv.request.motor_id = 0x05u;
    setModeSrv.request.command = can_driver::MotorCommand::Request::CMD_SET_MODE;
    setModeSrv.request.value = 1.0;
    ASSERT_TRUE(client.call(setModeSrv));
    EXPECT_FALSE(setModeSrv.response.success);
    EXPECT_NE(setModeSrv.response.message.find("disabled first"), std::string::npos);

    spinner.stop();
}

TEST_F(CanDriverHWSmokeTest, SetModeServiceAllowsMtSwitchWhileEnabledWithoutFreshPositionFeedback)
{
    auto fakeDm = std::make_shared<FakeDeviceManager>();
    CanDriverHW hw(fakeDm);

    ros::NodeHandle nh;
    ros::NodeHandle pnh(uniqueNs("can_driver_hw_smoke_set_mode_mt_enabled_allowed"));

    pnh.setParam("joints", makeSingleVelocityJoint());
    pnh.setParam("motor_state_period_sec", 0.05);

    ASSERT_TRUE(hw.init(nh, pnh));
    enterRunning(hw);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::ServiceClient client = nh.serviceClient<can_driver::MotorCommand>(
        pnh.resolveName("motor_command"));
    ASSERT_TRUE(client.waitForExistence(ros::Duration(1.0)));

    // 仅给 enabled 反馈，不给 positionValid，新逻辑下 MT 切模式应允许通过。
    setFreshEnabledFeedback(*fakeDm, "fake0", CanType::MT, static_cast<MotorID>(0x141), true);

    can_driver::MotorCommand setModeSrv;
    setModeSrv.request.motor_id = 0x141u;
    setModeSrv.request.command = can_driver::MotorCommand::Request::CMD_SET_MODE;
    setModeSrv.request.value = 0.0;
    ASSERT_TRUE(client.call(setModeSrv));
    EXPECT_TRUE(setModeSrv.response.success) << setModeSrv.response.message;
    EXPECT_EQ(fakeDm->protocol()->setModeCalls(), 1);
    EXPECT_EQ(fakeDm->protocol()->lastModeMotor(), 0x141u);
    EXPECT_EQ(fakeDm->protocol()->lastMode(), CanProtocol::MotorMode::Position);
    EXPECT_EQ(fakeDm->protocol()->positionCalls(), 0);

    spinner.stop();
}

TEST_F(CanDriverHWSmokeTest, ResumeAllowsAlignedCspTargetWithoutCommandChange)
{
    auto fakeDm = std::make_shared<FakeDeviceManager>();
    fakeDm->protocol()->setFeedbackPosition(1024);

    CanDriverHW hw(fakeDm);

    ros::NodeHandle nh;
    ros::NodeHandle pnh(uniqueNs("can_driver_hw_smoke_resume_aligned_csp"));

    pnh.setParam("joints", makeSingleCspJoint());
    pnh.setParam("motor_state_period_sec", 0.05);

    ASSERT_TRUE(hw.init(nh, pnh));

    const auto initResult = hw.operationalCoordinator().RequestInit("fake0", false);
    ASSERT_TRUE(initResult.ok) << initResult.message;

    auto *posIface = hw.get<hardware_interface::PositionJointInterface>();
    ASSERT_NE(posIface, nullptr);

    auto handle = posIface->getHandle("test_arm");
    const double alignedTarget = 1024.0 * (2.0 * M_PI / 65536.0);
    handle.setCommand(alignedTarget);

    const auto enableResult = hw.operationalCoordinator().RequestEnable();
    ASSERT_TRUE(enableResult.ok) << enableResult.message;

    const auto releaseResult = hw.operationalCoordinator().RequestRelease();
    ASSERT_TRUE(releaseResult.ok) << releaseResult.message;

    hw.write(ros::Time::now(), ros::Duration(0.01));

    EXPECT_EQ(fakeDm->protocol()->quickPositionCalls(), 1);
    EXPECT_EQ(fakeDm->protocol()->lastQuickPositionMotor(), 0x05u);
    EXPECT_EQ(fakeDm->protocol()->lastQuickPosition(), 1024);
}

TEST_F(CanDriverHWSmokeTest, CspReleaseRequiresSharedStateModeMatchBeforeRunning)
{
    auto fakeDm = std::make_shared<FakeDeviceManager>(true);
    fakeDm->setSyncSharedFeedbackFromProtocol(false);
    CanDriverHW hw(fakeDm);

    ros::NodeHandle nh;
    ros::NodeHandle pnh(uniqueNs("can_driver_hw_smoke_csp_lifecycle"));

    pnh.setParam("joints", makeSingleCspJoint());
    pnh.setParam("motor_state_period_sec", 0.05);

    ASSERT_TRUE(hw.init(nh, pnh));

    auto &coordinator = hw.operationalCoordinator();
    const auto initResult = coordinator.RequestInit("fake0", false);
    ASSERT_TRUE(initResult.ok) << initResult.message;
    EXPECT_EQ(coordinator.mode(), can_driver::SystemOpMode::Standby);

    const auto enableResult = coordinator.RequestEnable();
    ASSERT_TRUE(enableResult.ok) << enableResult.message;
    EXPECT_EQ(coordinator.mode(), can_driver::SystemOpMode::Armed);

    const auto axisKey = can_driver::MakeAxisKey("fake0", CanType::PP, static_cast<MotorID>(0x05));
    fakeDm->sharedState()->mutateDeviceHealth(
        "fake0",
        [](can_driver::SharedDriverState::DeviceHealthState *health) {
            health->transportReady = true;
        });
    fakeDm->sharedState()->mutateAxisCommand(
        axisKey,
        [](can_driver::SharedDriverState::AxisCommandState *command) {
            command->desiredMode = CanProtocol::MotorMode::CSP;
            command->desiredModeValid = true;
            command->valid = false;
        });
    fakeDm->sharedState()->setAxisIntent(axisKey, can_driver::AxisIntent::Enable);
    fakeDm->sharedState()->mutateAxisFeedback(
        axisKey,
        [](can_driver::SharedDriverState::AxisFeedbackState *feedback) {
            feedback->feedbackSeen = true;
            feedback->enabled = true;
            feedback->mode = CanProtocol::MotorMode::Position;
            feedback->lastRxSteadyNs = can_driver::SharedDriverSteadyNowNs();
        });

    const auto blockedRelease = coordinator.RequestRelease();
    EXPECT_FALSE(blockedRelease.ok);
    EXPECT_EQ(blockedRelease.message, "Mode not ready.");
    EXPECT_EQ(coordinator.mode(), can_driver::SystemOpMode::Armed);

    fakeDm->sharedState()->mutateAxisFeedback(
        axisKey,
        [](can_driver::SharedDriverState::AxisFeedbackState *feedback) {
            feedback->mode = CanProtocol::MotorMode::CSP;
            feedback->lastRxSteadyNs = can_driver::SharedDriverSteadyNowNs();
        });

    const auto releaseResult = coordinator.RequestRelease();
    EXPECT_TRUE(releaseResult.ok) << releaseResult.message;
    EXPECT_EQ(coordinator.mode(), can_driver::SystemOpMode::Running);
}

} // namespace
