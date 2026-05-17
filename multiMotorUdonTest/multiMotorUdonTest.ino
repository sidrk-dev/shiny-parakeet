#include <Arduino.h>
#include <EEPROM.h>
#include <Wire.h>
#include <stdint.h>
#include <string.h>

#define UDON_CANBUS_SPI_PICO_POLLING 1
#include <Udon.hpp>
#include "AbsoluteEncoder.hpp"
#include "MotorController.hpp"
#include "RoboMasterArmProtocol.hpp"

namespace Protocol = RoboMasterArmProtocol;

static constexpr uint8_t kMotorCount = Protocol::kMaxMotors;
static constexpr unsigned long kControlIntervalUs = 1000;
static constexpr uint32_t kSerialBaud = 921600;
static constexpr uint32_t kSerialWatchdogMs = 100;

// Seeed XIAO RP2350 + MCP2515 CAN pins. These match the working singleM3508 wiring.
static constexpr uint8_t kCanCsPin = D3;
static constexpr uint8_t kCanMosiPin = D10;
static constexpr uint8_t kCanMisoPin = D9;
static constexpr uint8_t kCanSckPin = D8;

// Verified AS5600/PCA9548A bus from encoderTest.ino.
static constexpr uint8_t kI2cSdaPin = 6;
static constexpr uint8_t kI2cSclPin = 7;
static constexpr uint32_t kI2cClockHz = 100000;
static constexpr uint8_t kMaxEncoderFailuresBeforeInvalid = 3;

#if defined(TEENSYDUINO)
static Udon::CanBusTeensy<CAN1>::Config makeCanConfig() {
    Udon::CanBusTeensy<CAN1>::Config config;
    config.transmitInterval = 1;
    config.canBaudrate = 1000000;
    return config;
}
static Udon::CanBusTeensy<CAN1> bus{makeCanConfig()};
#elif defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_ARCH_RP2350)
static Udon::CanBusSpi::Config makeCanConfig() {
    Udon::CanBusSpi::Config config;
    config.channel = spi0;
    config.cs = kCanCsPin;
    config.mosi = kCanMosiPin;
    config.miso = kCanMisoPin;
    config.sck = kCanSckPin;
    config.spiClock = 8000000;
    config.transmitInterval = 1;
    config.canBaudrate = CAN_1000KBPS;
    config.mcpClock = MCP_8MHZ;
    return config;
}
static Udon::CanBusSpi bus{makeCanConfig()};
#else
#error "Add a CAN bus definition for this Arduino core."
#endif

static AbsoluteEncoderMux encoderMux{Wire1, kI2cSdaPin, kI2cSclPin, kI2cClockHz};
static Udon::LoopCycleController loopCtrl{kControlIntervalUs};

static Udon::RoboMasterC620 m1{bus, 1};
static Udon::RoboMasterC610 m2{bus, 2};
static Udon::RoboMasterC620 m3{bus, 3}, m4{bus, 4}, m5{bus, 5}, m6{bus, 6}, m7{bus, 7}, m8{bus, 8};

static const MotorController::Config kCfg = {
    {2.0, 0.0, 0.02},
    {2.0, 0.0, 0.03},
    500.0,
    800.0,
    6000.0
};

static MotorController c1{1, m1, kCfg, kControlIntervalUs};
static MotorController c2{2, m2, kCfg, kControlIntervalUs};
static MotorController c3{3, m3, kCfg, kControlIntervalUs};
static MotorController c4{4, m4, kCfg, kControlIntervalUs};
static MotorController c5{5, m5, kCfg, kControlIntervalUs};
static MotorController c6{6, m6, kCfg, kControlIntervalUs};
static MotorController c7{7, m7, kCfg, kControlIntervalUs};
static MotorController c8{8, m8, kCfg, kControlIntervalUs};
static MotorController* motors[kMotorCount] = {&c1, &c2, &c3, &c4, &c5, &c6, &c7, &c8};

struct EncoderState {
    bool configured = false;
    uint8_t muxChannel = 0;
    bool valid = false;
    uint8_t consecutiveFailures = 0;
    uint8_t lastError = 0;
    uint16_t raw = AbsoluteEncoderMux::kInvalidRaw;
    double rawDeg = 0.0;
    double zeroOffsetDeg = 0.0;
    double positionDeg = 0.0;
    uint32_t lastSampleMs = 0;
};

struct PersistentConfig {
    uint32_t magic;
    uint16_t version;
    uint16_t sizeBytes;
    uint8_t encoderConfigured[kMotorCount];
    uint8_t muxChannel[kMotorCount];
    uint8_t motorType[kMotorCount]; // 1=M3508/C620, 2=M2006/C610
    uint8_t reserved[kMotorCount];
    float zeroOffsetDeg[kMotorCount];
    float positionPidP[kMotorCount];
    float positionPidI[kMotorCount];
    float positionPidD[kMotorCount];
    float velocityLimitRpm[kMotorCount];
};

static constexpr uint32_t kConfigMagic = 0x524D4152UL; // "RMAR"
static constexpr uint16_t kConfigVersion = 3;
static EncoderState encoders[kMotorCount];
static PersistentConfig persistentConfig;
static uint8_t nextEncoderToPoll = 0;

static uint8_t rxEncoded[Protocol::kMaxEncodedBytes];
static uint8_t rxDecoded[Protocol::kMaxPacketBytes];
static uint8_t txEncoded[Protocol::kMaxEncodedBytes];
static size_t rxEncodedLength = 0;
static uint32_t txSequence = 1;
static uint32_t lastValidCommandMs = 0;
static bool watchdogTripped = true;

static void stopAllMotors() {
    for (uint8_t i = 0; i < kMotorCount; ++i) {
        motors[i]->stop();
    }
}

static void setDefaultPersistentConfig() {
    persistentConfig.magic = kConfigMagic;
    persistentConfig.version = kConfigVersion;
    persistentConfig.sizeBytes = sizeof(PersistentConfig);
    for (uint8_t i = 0; i < kMotorCount; ++i) {
        persistentConfig.encoderConfigured[i] = (i == 0) ? 1 : 0;
        persistentConfig.muxChannel[i] = i;
        persistentConfig.motorType[i] = (i == 1) ? 2 : 1;
        persistentConfig.reserved[i] = 0;
        persistentConfig.zeroOffsetDeg[i] = 0.0f;
        persistentConfig.positionPidP[i] = static_cast<float>(kCfg.positionGains.p);
        persistentConfig.positionPidI[i] = static_cast<float>(kCfg.positionGains.i);
        persistentConfig.positionPidD[i] = static_cast<float>(kCfg.positionGains.d);
        persistentConfig.velocityLimitRpm[i] = static_cast<float>(kCfg.maxPositionVelocityRpm);
    }
    persistentConfig.muxChannel[0] = 0;
}

static void savePersistentConfig() {
    persistentConfig.magic = kConfigMagic;
    persistentConfig.version = kConfigVersion;
    persistentConfig.sizeBytes = sizeof(PersistentConfig);
    for (uint8_t i = 0; i < kMotorCount; ++i) {
        const MotorController::PidGains gains = motors[i]->getPositionGains();
        persistentConfig.encoderConfigured[i] = encoders[i].configured ? 1 : 0;
        persistentConfig.muxChannel[i] = encoders[i].muxChannel;
        persistentConfig.zeroOffsetDeg[i] = static_cast<float>(encoders[i].zeroOffsetDeg);
        persistentConfig.positionPidP[i] = static_cast<float>(gains.p);
        persistentConfig.positionPidI[i] = static_cast<float>(gains.i);
        persistentConfig.positionPidD[i] = static_cast<float>(gains.d);
        persistentConfig.velocityLimitRpm[i] = static_cast<float>(motors[i]->getLimitV());
    }
    EEPROM.put(0, persistentConfig);
    EEPROM.commit();
}

static void applyPersistentConfig() {
    for (uint8_t i = 0; i < kMotorCount; ++i) {
        encoders[i].zeroOffsetDeg = AbsoluteEncoderMux::normalize0To360(persistentConfig.zeroOffsetDeg[i]);
        encoders[i].muxChannel = persistentConfig.muxChannel[i] <= 7 ? persistentConfig.muxChannel[i] : i;
        encoders[i].configured = persistentConfig.encoderConfigured[i] != 0 && encoders[i].muxChannel <= 7;
        encoders[i].valid = false;
        encoders[i].consecutiveFailures = 0;
        encoders[i].lastError = 0;

        motors[i]->setMotorType(persistentConfig.motorType[i] == 1);
        motors[i]->setLimitV(persistentConfig.velocityLimitRpm[i]);
        motors[i]->setPositionGains(
            persistentConfig.positionPidP[i],
            persistentConfig.positionPidI[i],
            persistentConfig.positionPidD[i]
        );
        motors[i]->enableAbsoluteFeedback(encoders[i].configured);
    }
}

static void loadPersistentConfig() {
    EEPROM.begin(sizeof(PersistentConfig));
    EEPROM.get(0, persistentConfig);
    if (persistentConfig.magic != kConfigMagic
        || persistentConfig.version != kConfigVersion
        || persistentConfig.sizeBytes != sizeof(PersistentConfig)) {
        setDefaultPersistentConfig();
        EEPROM.put(0, persistentConfig);
        EEPROM.commit();
    }
    applyPersistentConfig();
}

static bool configureEncoder(uint8_t motorId, uint8_t muxChannel) {
    if (motorId < 1 || motorId > kMotorCount || muxChannel > 7) {
        return false;
    }

    EncoderState& encoder = encoders[motorId - 1];
    encoder.configured = true;
    encoder.muxChannel = muxChannel;
    encoder.valid = false;
    encoder.consecutiveFailures = 0;
    encoder.lastError = 0;
    motors[motorId - 1]->enableAbsoluteFeedback(true);
    return true;
}

static bool disableEncoder(uint8_t motorId) {
    if (motorId < 1 || motorId > kMotorCount) {
        return false;
    }

    EncoderState& encoder = encoders[motorId - 1];
    encoder.configured = false;
    encoder.valid = false;
    motors[motorId - 1]->enableAbsoluteFeedback(false);
    return true;
}

static void pollEncoder(uint8_t index) {
    EncoderState& encoder = encoders[index];
    if (!encoder.configured) {
        return;
    }

    uint16_t raw = AbsoluteEncoderMux::kInvalidRaw;
    const uint32_t now = millis();
    if (encoderMux.readRawAngle(encoder.muxChannel, raw)) {
        encoder.raw = raw;
        encoder.rawDeg = AbsoluteEncoderMux::rawToDegrees(raw);
        encoder.positionDeg = AbsoluteEncoderMux::normalize0To360(encoder.rawDeg - encoder.zeroOffsetDeg);
        encoder.valid = true;
        encoder.consecutiveFailures = 0;
        encoder.lastError = 0;
        encoder.lastSampleMs = now;
        motors[index]->setAbsolutePositionFeedbackDeg(encoder.positionDeg, true, now);
        return;
    }

    encoder.lastError = encoderMux.getLastError();
    if (encoder.consecutiveFailures < 255) {
        encoder.consecutiveFailures++;
    }

    if (encoder.consecutiveFailures >= kMaxEncoderFailuresBeforeInvalid) {
        encoder.valid = false;
        motors[index]->setAbsolutePositionFeedbackDeg(encoder.positionDeg, false, now);
        encoderMux.recover();
    }
}

static void pollOneConfiguredEncoder() {
    for (uint8_t attempts = 0; attempts < kMotorCount; ++attempts) {
        const uint8_t index = nextEncoderToPoll;
        nextEncoderToPoll = static_cast<uint8_t>((nextEncoderToPoll + 1) % kMotorCount);
        if (encoders[index].configured) {
            pollEncoder(index);
            return;
        }
    }
}

static void snapshotAllConfiguredEncoders() {
    for (uint8_t i = 0; i < kMotorCount; ++i) {
        if (encoders[i].configured) {
            pollEncoder(i);
        }
    }
}

static bool setEncoderZero(uint8_t motorId) {
    if (motorId < 1 || motorId > kMotorCount) {
        return false;
    }

    EncoderState& encoder = encoders[motorId - 1];
    if (!encoder.configured || !encoder.valid) {
        return false;
    }

    encoder.zeroOffsetDeg = encoder.rawDeg;
    encoder.positionDeg = 0.0;
    motors[motorId - 1]->setAbsolutePositionFeedbackDeg(0.0, true, millis());
    motors[motorId - 1]->rebasePositionTargetToCurrent();
    return true;
}

static void sendEncodedPacket(const void* packet, size_t packetSize) {
    const size_t encodedLength = Protocol::cobsEncode(
        reinterpret_cast<const uint8_t*>(packet),
        packetSize,
        txEncoded,
        sizeof(txEncoded)
    );
    if (encodedLength == 0) {
        return;
    }
    Serial.write(txEncoded, encodedLength);
    Serial.write(static_cast<uint8_t>(0));
}

static void sendTelemetryPacket(Protocol::TelemetryPacket& packet, uint32_t sequence) {
    Protocol::finalizePacket(packet, Protocol::PacketType::Telemetry, sequence);
    sendEncodedPacket(&packet, sizeof(packet));
}

static void sendConfigAckPacket(Protocol::ConfigAckPacket& packet, uint32_t sequence) {
    Protocol::finalizePacket(packet, Protocol::PacketType::ConfigAck, sequence);
    sendEncodedPacket(&packet, sizeof(packet));
}

static uint8_t protocolModeFromController(MotorController::Mode mode) {
    switch (mode) {
        case MotorController::Mode::Velocity: return static_cast<uint8_t>(Protocol::MotorMode::Velocity);
        case MotorController::Mode::Position: return static_cast<uint8_t>(Protocol::MotorMode::Position);
        case MotorController::Mode::Current: return static_cast<uint8_t>(Protocol::MotorMode::Current);
        case MotorController::Mode::Stopped:
        default: return static_cast<uint8_t>(Protocol::MotorMode::Stop);
    }
}

static void sendTelemetry(uint32_t requestSequence) {
    snapshotAllConfiguredEncoders();

    Protocol::TelemetryPacket packet;
    memset(&packet, 0, sizeof(packet));
    const uint32_t now = millis();
    packet.mcu_time_ms = now;
    packet.last_command_age_ms = lastValidCommandMs == 0 ? 0xFFFFFFFFUL : now - lastValidCommandMs;
    packet.motor_count = kMotorCount;
    packet.watchdog_tripped = watchdogTripped ? 1 : 0;

    for (uint8_t i = 0; i < kMotorCount; ++i) {
        packet.active[i] = motors[i]->isActive() ? 1 : 0;
        packet.mode[i] = protocolModeFromController(motors[i]->getMode());
        packet.encoder_configured[i] = encoders[i].configured ? 1 : 0;
        packet.encoder_valid[i] = encoders[i].valid ? 1 : 0;
        packet.encoder_channel[i] = encoders[i].configured ? encoders[i].muxChannel : 255;
        packet.i2c_error[i] = encoders[i].lastError;
        packet.position_deg[i] = static_cast<float>(motors[i]->getPositionFeedbackDeg());
        packet.velocity_rpm[i] = static_cast<float>(motors[i]->getOutputVelocity());
        packet.current_ma[i] = static_cast<float>(motors[i]->getCurr());
        packet.temperature_c[i] = static_cast<float>(motors[i]->getTemp());
    }

    sendTelemetryPacket(packet, requestSequence == 0 ? txSequence++ : requestSequence);
}

static void sendConfigAck(Protocol::ConfigCommand command, uint8_t motorId, Protocol::ConfigStatus status, uint32_t requestSequence) {
    Protocol::ConfigAckPacket packet;
    memset(&packet, 0, sizeof(packet));
    packet.command = static_cast<uint8_t>(command);
    packet.motor_id = motorId;
    packet.status = static_cast<uint8_t>(status);
    sendConfigAckPacket(packet, requestSequence == 0 ? txSequence++ : requestSequence);
}

static void applyCommandPacket(const Protocol::CommandPacket& packet) {
    const uint8_t count = packet.motor_count <= kMotorCount ? packet.motor_count : kMotorCount;
    for (uint8_t i = 0; i < kMotorCount; ++i) {
        const Protocol::MotorMode mode = i < count
            ? static_cast<Protocol::MotorMode>(packet.mode[i])
            : Protocol::MotorMode::Stop;

        switch (mode) {
            case Protocol::MotorMode::Position:
                motors[i]->setTargetPositionDeg(packet.target_position_deg[i]);
                break;
            case Protocol::MotorMode::Velocity:
                motors[i]->setTargetVelocity(packet.target_velocity_rpm[i]);
                break;
            case Protocol::MotorMode::Current:
                motors[i]->setTargetCurrent(packet.target_current_ma[i]);
                break;
            case Protocol::MotorMode::Disabled:
            case Protocol::MotorMode::Stop:
            default:
                motors[i]->stop();
                break;
        }
    }

    lastValidCommandMs = millis();
    watchdogTripped = false;
}

static void applyConfigPacket(const Protocol::ConfigPacket& packet) {
    const Protocol::ConfigCommand command = static_cast<Protocol::ConfigCommand>(packet.command);
    const uint8_t motorId = packet.motor_id;
    if (motorId < 1 || motorId > kMotorCount) {
        sendConfigAck(command, motorId, Protocol::ConfigStatus::BadMotorId, packet.header.sequence);
        return;
    }

    Protocol::ConfigStatus status = Protocol::ConfigStatus::Ok;
    switch (command) {
        case Protocol::ConfigCommand::EnableEncoder:
            if (packet.encoder_channel > 7 || !configureEncoder(motorId, packet.encoder_channel)) {
                status = Protocol::ConfigStatus::BadChannel;
            }
            break;
        case Protocol::ConfigCommand::DisableEncoder:
            disableEncoder(motorId);
            break;
        case Protocol::ConfigCommand::ZeroCurrentPosition:
            if (!setEncoderZero(motorId)) {
                status = Protocol::ConfigStatus::EncoderInvalid;
            }
            break;
        case Protocol::ConfigCommand::SetPositionPid:
            motors[motorId - 1]->setPositionGains(packet.position_pid_p, packet.position_pid_i, packet.position_pid_d);
            break;
        case Protocol::ConfigCommand::SetMotorType:
            persistentConfig.motorType[motorId - 1] = packet.motor_type == 2 ? 2 : 1;
            motors[motorId - 1]->setMotorType(persistentConfig.motorType[motorId - 1] == 1);
            break;
        case Protocol::ConfigCommand::SetVelocityLimit:
            motors[motorId - 1]->setLimitV(packet.velocity_limit_rpm);
            break;
        case Protocol::ConfigCommand::SaveConfig:
            break;
        default:
            status = Protocol::ConfigStatus::BadCommand;
            break;
    }

    if (status == Protocol::ConfigStatus::Ok) {
        savePersistentConfig();
    }
    sendConfigAck(command, motorId, status, packet.header.sequence);
}

static void handleDecodedPacket(const uint8_t* data, size_t length) {
    if (length < sizeof(Protocol::PacketHeader)) {
        return;
    }

    const Protocol::PacketHeader* header = reinterpret_cast<const Protocol::PacketHeader*>(data);
    const Protocol::PacketType type = static_cast<Protocol::PacketType>(header->type);
    if (header->version != Protocol::kProtocolVersion || Protocol::expectedPacketSize(type) != length) {
        return;
    }

    switch (type) {
        case Protocol::PacketType::Command: {
            Protocol::CommandPacket packet;
            memcpy(&packet, data, sizeof(packet));
            if (Protocol::validatePacket(packet, Protocol::PacketType::Command)) {
                applyCommandPacket(packet);
            }
            break;
        }
        case Protocol::PacketType::Config: {
            Protocol::ConfigPacket packet;
            memcpy(&packet, data, sizeof(packet));
            if (Protocol::validatePacket(packet, Protocol::PacketType::Config)) {
                applyConfigPacket(packet);
            }
            break;
        }
        case Protocol::PacketType::TelemetryRequest: {
            Protocol::TelemetryRequestPacket packet;
            memcpy(&packet, data, sizeof(packet));
            if (Protocol::validatePacket(packet, Protocol::PacketType::TelemetryRequest)) {
                sendTelemetry(packet.header.sequence);
            }
            break;
        }
        case Protocol::PacketType::Halt:
            stopAllMotors();
            watchdogTripped = true;
            break;
        default:
            break;
    }
}

static void processSerialProtocol() {
    while (Serial.available() > 0) {
        const int next = Serial.read();
        if (next < 0) {
            return;
        }

        const uint8_t byte = static_cast<uint8_t>(next);
        if (byte == 0) {
            if (rxEncodedLength > 0) {
                const size_t decodedLength = Protocol::cobsDecode(
                    rxEncoded,
                    rxEncodedLength,
                    rxDecoded,
                    sizeof(rxDecoded)
                );
                if (decodedLength > 0) {
                    handleDecodedPacket(rxDecoded, decodedLength);
                }
                rxEncodedLength = 0;
            }
            continue;
        }

        if (rxEncodedLength < sizeof(rxEncoded)) {
            rxEncoded[rxEncodedLength++] = byte;
        } else {
            rxEncodedLength = 0;
        }
    }
}

static void enforceSerialWatchdog() {
    const uint32_t now = millis();
    if (lastValidCommandMs == 0 || now - lastValidCommandMs > kSerialWatchdogMs) {
        if (!watchdogTripped) {
            stopAllMotors();
        }
        watchdogTripped = true;
    }
}

void setup() {
    Serial.begin(kSerialBaud);

    encoderMux.begin();
    loadPersistentConfig();
    bus.begin();
    stopAllMotors();
}

void loop() {
    bus.update();
    pollOneConfiguredEncoder();
    processSerialProtocol();
    enforceSerialWatchdog();

    for (uint8_t i = 0; i < kMotorCount; ++i) {
        motors[i]->update();
    }

    loopCtrl.update();
}
