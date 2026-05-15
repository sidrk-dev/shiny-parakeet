#pragma once

#include <Arduino.h>
#include <math.h>
#include <stdint.h>
#include <string.h>
#include <Udon.hpp>

class MotorController
{
public:
    enum class Mode : uint8_t { Stopped, Velocity, Position, Current };

    struct PidGains { double p, i, d; };
    struct Config {
        PidGains positionGains;
        PidGains velocityGains;
        double maxPositionVelocityRpm;
        double positionIntegralLimitRpm;
        double velocityIntegralLimitMilliamp;
    };

    MotorController(uint8_t motorId, Udon::RoboMasterBase& motor, const Config& config, unsigned long controlIntervalUs)
        : motorId_(motorId)
        , motor_(motor)
        , positionPid_(config.positionGains.p, config.positionGains.i, config.positionGains.d, controlIntervalUs, config.positionIntegralLimitRpm)
        , velocityPid_(config.velocityGains.p, config.velocityGains.i, config.velocityGains.d, controlIntervalUs, config.velocityIntegralLimitMilliamp)
        , positionGains_(config.positionGains)
    {
        setMotorType(false); // Default to M2006/C610 gearing until configured.
        maxOutputVelocityRpm_ = config.maxPositionVelocityRpm;
        stop();
    }

    void setMotorType(bool isM3508) {
        if (isM3508) {
            gearRatio_ = 19.0;
            motorType_ = "M3508";
        } else {
            gearRatio_ = 36.0;
            motorType_ = "M2006";
        }
        if (mode_ == Mode::Position && !useAbsoluteFeedback_) {
            targetOutputPositionDeg_ = getCanPositionDeg();
            clearControlState();
        }
    }

    bool isActive() const {
        const uint32_t lastSeen = motor_.getLastReceivedMs();
        return lastSeen != 0 && (millis() - lastSeen < kMotorFeedbackTimeoutMs);
    }

    void setTargetVelocity(double rpm) {
        if (mode_ != Mode::Velocity) {
            clearControlState();
        }
        targetOutputVelocityRpm_ = Udon::Constrain(rpm, -maxOutputVelocityRpm_, maxOutputVelocityRpm_);
        targetCurrentMilliamp_ = 0.0;
        mode_ = Mode::Velocity;
    }

    void setTargetPositionDeg(double deg) {
        targetOutputPositionDeg_ = useAbsoluteFeedback_ ? normalize0To360(deg) : deg;
        targetOutputVelocityRpm_ = 0.0;
        targetCurrentMilliamp_ = 0.0;
        mode_ = Mode::Position;
        clearControlState();
    }

    void setTargetPositionRad(double rad) {
        setTargetPositionDeg(rad * kRadToDeg);
    }

    void rebasePositionTargetToCurrent() {
        targetOutputPositionDeg_ = useAbsoluteFeedback_ ? normalize0To360(getPositionFeedbackDeg()) : getPositionFeedbackDeg();
        clearControlState();
    }

    void setTargetCurrent(double mA) {
        if (mode_ != Mode::Current) {
            clearControlState();
        }
        targetCurrentMilliamp_ = mA;
        targetOutputVelocityRpm_ = 0.0;
        mode_ = Mode::Current;
    }

    void stop() {
        mode_ = Mode::Stopped;
        targetOutputVelocityRpm_ = 0.0;
        targetCurrentMilliamp_ = 0.0;
        clearControlState();
        motor_.setCurrent(0);
    }

    /// Enable AS5600/PCA9548A absolute feedback for this joint. Samples are
    /// supplied by main.ino so MotorController stays independent of the I2C bus.
    void enableAbsoluteFeedback(bool enable) {
        if (useAbsoluteFeedback_ == enable) {
            return;
        }

        const double positionBeforeTransitionDeg = getPositionFeedbackDeg();
        useAbsoluteFeedback_ = enable;
        absoluteFeedbackValid_ = false;
        lastAbsoluteSampleMs_ = 0;
        if (mode_ == Mode::Position) {
            targetOutputPositionDeg_ = enable ? positionBeforeTransitionDeg : getPositionFeedbackDeg();
        }
        clearControlState();
    }

    void setAbsolutePositionFeedbackDeg(double positionDeg, bool valid, uint32_t sampleMs) {
        const bool hadValidFeedback = hasValidAbsoluteFeedback();

        if (valid) {
            lastAbsolutePositionDeg_ = normalize0To360(positionDeg);
            absoluteFeedbackValid_ = true;
            lastAbsoluteSampleMs_ = sampleMs;

            if (useAbsoluteFeedback_ && mode_ == Mode::Position && !hadValidFeedback) {
                targetOutputPositionDeg_ = lastAbsolutePositionDeg_;
                clearControlState();
            }
            return;
        }

        if (useAbsoluteFeedback_ && mode_ == Mode::Position && hadValidFeedback) {
            targetOutputPositionDeg_ = lastAbsolutePositionDeg_;
            clearControlState();
        }
        absoluteFeedbackValid_ = false;
        lastAbsoluteSampleMs_ = sampleMs;
    }

    void setAbsolutePositionFeedback(double positionRad, bool valid, uint32_t sampleMs) {
        setAbsolutePositionFeedbackDeg(positionRad * kRadToDeg, valid, sampleMs);
    }

    bool hasValidAbsoluteFeedback() const {
        return useAbsoluteFeedback_
            && absoluteFeedbackValid_
            && lastAbsoluteSampleMs_ != 0
            && (millis() - lastAbsoluteSampleMs_ <= kAbsoluteFeedbackTimeoutMs);
    }

    void setPositionGains(double p, double i, double d) {
        positionGains_ = {p, i, d};
        positionPid_.setParam({p, i, d});
        positionPid_.clearPower();
    }

    void update() {
        if (!isActive()) {
            motor_.setCurrent(0);
            clearControlState();
            return;
        }

        if (mode_ == Mode::Stopped) {
            motor_.setCurrent(0);
            return;
        }

        if (mode_ == Mode::Current) {
            applyCurrentLimit(targetCurrentMilliamp_);
            return;
        }

        double internalTargetRpm = 0.0;

        if (mode_ == Mode::Position) {
            if (useAbsoluteFeedback_ && !hasValidAbsoluteFeedback()) {
                motor_.setCurrent(0);
                clearControlState();
                return;
            }

            const double currentPosDeg = getPositionFeedbackDeg();
            const double targetDeg = useAbsoluteFeedback_
                ? currentPosDeg + wrapTo180(targetOutputPositionDeg_ - currentPosDeg)
                : targetOutputPositionDeg_;
            const double outputVelocityRpm = positionPid_(currentPosDeg, targetDeg, -maxOutputVelocityRpm_, maxOutputVelocityRpm_);
            internalTargetRpm = outputVelocityRpm * gearRatio_;
        } else {
            internalTargetRpm = targetOutputVelocityRpm_ * gearRatio_;
        }

        applyCurrentLimit(velocityPid_(static_cast<double>(motor_.getVelocity()), internalTargetRpm));
    }

    uint8_t getId() const { return motorId_; }
    Mode getMode() const { return mode_; }
    const char* getMotorType() const { return motorType_; }
    bool isUsingExternal() const { return useAbsoluteFeedback_; }
    bool isExternalValid() const { return hasValidAbsoluteFeedback(); }

    double getCanPositionDeg() const { return (motor_.getAngle() / gearRatio_) * kRadToDeg; }
    double getOutputPositionDeg() const { return getPositionFeedbackDeg(); }
    double getExternalPositionDeg() const { return lastAbsolutePositionDeg_; }
    double getPositionFeedbackDeg() const {
        return useAbsoluteFeedback_ ? lastAbsolutePositionDeg_ : getCanPositionDeg();
    }

    double getOutputPositionRad() const { return getCanPositionDeg() * kDegToRad; }
    double getExternalPositionRad() const { return lastAbsolutePositionDeg_ * kDegToRad; }
    double getPositionFeedbackRad() const { return getPositionFeedbackDeg() * kDegToRad; }

    double getOutputVelocity() const { return static_cast<double>(motor_.getVelocity()) / gearRatio_; }
    double getTargetV() const { return targetOutputVelocityRpm_; }
    double getTargetP() const { return targetOutputPositionDeg_; }
    double getTargetPDeg() const { return targetOutputPositionDeg_; }
    double getLimitV() const { return maxOutputVelocityRpm_; }
    uint8_t getTemp() const { return motor_.getTemperature(); }
    int16_t getCurr() const { return motor_.getTorqueCurrent(); }

    PidGains getPositionGains() const { return positionGains_; }

    void setGearing(double g) {
        gearRatio_ = g;
        if (mode_ == Mode::Position && !useAbsoluteFeedback_) {
            targetOutputPositionDeg_ = getCanPositionDeg();
            clearControlState();
        }
    }
    void setLimitV(double l) { maxOutputVelocityRpm_ = fabs(l); }

private:
    static constexpr uint32_t kMotorFeedbackTimeoutMs = 1000;
    static constexpr uint32_t kAbsoluteFeedbackTimeoutMs = 80;
    static constexpr double kDegToRad = 0.01745329251994329577;
    static constexpr double kRadToDeg = 57.2957795130823208768;

    static double normalize0To360(double deg) {
        double normalized = fmod(deg, 360.0);
        if (normalized < 0.0) normalized += 360.0;
        return normalized;
    }

    static double wrapTo180(double deg) {
        double wrapped = fmod(deg + 180.0, 360.0);
        if (wrapped < 0.0) wrapped += 360.0;
        return wrapped - 180.0;
    }

    void clearControlState() {
        positionPid_.clearPower();
        velocityPid_.clearPower();
    }

    void applyCurrentLimit(double mA) {
        const Udon::Range<int16_t> range = motor_.getCurrentRange();
        const double clipped = Udon::Constrain(mA, static_cast<double>(range.min), static_cast<double>(range.max));
        motor_.setCurrent(static_cast<int16_t>(clipped));
    }

    uint8_t motorId_;
    Udon::RoboMasterBase& motor_;
    Udon::PidController positionPid_, velocityPid_;
    PidGains positionGains_;

    Mode mode_ = Mode::Stopped;
    double gearRatio_ = 1.0;
    double maxOutputVelocityRpm_ = 0.0;
    double targetOutputVelocityRpm_ = 0.0;
    double targetOutputPositionDeg_ = 0.0;
    double targetCurrentMilliamp_ = 0.0;
    const char* motorType_ = "Unknown";

    bool useAbsoluteFeedback_ = false;
    bool absoluteFeedbackValid_ = false;
    double lastAbsolutePositionDeg_ = 0.0;
    uint32_t lastAbsoluteSampleMs_ = 0;
};
