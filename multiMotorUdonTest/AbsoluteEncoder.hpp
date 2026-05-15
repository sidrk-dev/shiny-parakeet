#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <stdint.h>

class AbsoluteEncoderMux
{
public:
    static constexpr uint8_t kMuxAddress = 0x70;
    static constexpr uint8_t kAs5600Address = 0x36;
    static constexpr uint8_t kRawAngleRegister = 0x0C;
    static constexpr uint16_t kInvalidRaw = 0xFFFF;
    static constexpr uint16_t kCountsPerRev = 4096;
    static constexpr double kTwoPi = 6.28318530717958647692;

    AbsoluteEncoderMux(TwoWire& wire, uint8_t sdaPin, uint8_t sclPin, uint32_t clockHz)
        : wire_(wire), sdaPin_(sdaPin), sclPin_(sclPin), clockHz_(clockHz)
    {
    }

    void begin() {
        wire_.setSDA(sdaPin_);
        wire_.setSCL(sclPin_);
        wire_.begin();
        wire_.setClock(clockHz_);
        currentChannel_ = 0xFF;
        lastError_ = 0;
    }

    bool selectChannel(uint8_t channel) {
        if (channel > 7) {
            lastError_ = 98;
            return false;
        }

        wire_.beginTransmission(kMuxAddress);
        wire_.write(static_cast<uint8_t>(1U << channel));
        const uint8_t error = wire_.endTransmission(true);
        lastError_ = error;
        if (error == 0) {
            currentChannel_ = channel;
            return true;
        }
        return false;
    }

    bool readRawAngle(uint8_t channel, uint16_t& raw) {
        raw = kInvalidRaw;
        if (!selectChannel(channel)) {
            return false;
        }

        // Standard stop-start transaction. The AS5600 register write is
        // terminated with STOP, then the two data bytes are requested.
        wire_.beginTransmission(kAs5600Address);
        wire_.write(kRawAngleRegister);
        const uint8_t error = wire_.endTransmission(true);
        lastError_ = error;
        if (error != 0) {
            return false;
        }

        if (wire_.requestFrom(kAs5600Address, static_cast<uint8_t>(2)) != 2) {
            lastError_ = 99;
            return false;
        }

        raw = static_cast<uint16_t>((wire_.read() << 8) | wire_.read()) & 0x0FFF;
        lastError_ = 0;
        return true;
    }

    void recover() {
        wire_.end();
        delayMicroseconds(100);
        begin();
    }

    uint8_t getLastError() const { return lastError_; }
    uint8_t getCurrentChannel() const { return currentChannel_; }

    static double rawToRadians(uint16_t raw) {
        return (static_cast<double>(raw) * kTwoPi) / static_cast<double>(kCountsPerRev);
    }

    static double rawToDegrees(uint16_t raw) {
        return (static_cast<double>(raw) * 360.0) / static_cast<double>(kCountsPerRev);
    }

    static double normalize0ToTwoPi(double rad) {
        while (rad >= kTwoPi) rad -= kTwoPi;
        while (rad < 0.0) rad += kTwoPi;
        return rad;
    }

    static double normalize0To360(double deg) {
        while (deg >= 360.0) deg -= 360.0;
        while (deg < 0.0) deg += 360.0;
        return deg;
    }

private:
    TwoWire& wire_;
    uint8_t sdaPin_;
    uint8_t sclPin_;
    uint32_t clockHz_;
    uint8_t currentChannel_ = 0xFF;
    uint8_t lastError_ = 0;
};
