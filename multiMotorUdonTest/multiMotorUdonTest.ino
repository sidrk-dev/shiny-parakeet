#include <Arduino.h>
#include <EEPROM.h>
#include <Wire.h>
#include <ctype.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define UDON_CANBUS_SPI_PICO_POLLING 1
#include <Udon.hpp>
#include "AbsoluteEncoder.hpp"
#include "MotorController.hpp"

static constexpr uint8_t kMotorCount = 8;
static constexpr unsigned long kControlIntervalUs = 1000;
static uint32_t kTelemetryIntervalMs = 200;

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

// One CAN bus can host up to 8 RoboMaster ESCs. Motor 1 is C620/M3508, motor 2
// is C610/M2006, and the remaining slots are ready for future joints.
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

struct EncoderCalibrationStorage {
    uint32_t magic;
    uint16_t version;
    uint16_t sizeBytes;
    uint8_t encoderConfigured[kMotorCount];
    uint8_t muxChannel[kMotorCount];
    float zeroOffsetDeg[kMotorCount];
    float positionPidP[kMotorCount];
    float positionPidI[kMotorCount];
    float positionPidD[kMotorCount];
};

static constexpr uint32_t kEncoderCalMagic = 0x41534346UL; // "ASCF"
static constexpr uint16_t kEncoderCalVersion = 2;
static EncoderState encoders[kMotorCount];
static EncoderCalibrationStorage encoderCal;
static uint8_t nextEncoderToPoll = 0;

static bool teleEnabled = true;
static bool humanMode = true;
static char cmdBuf[96];
static size_t cmdLen = 0;
static uint32_t lastTeleMs = 0;

static bool parseD(const char* token, double& value) {
    if (!token) return false;
    char* end = nullptr;
    value = strtod(token, &end);
    while (*end == ' ' || *end == '\t' || *end == '\r' || *end == '\n') end++;
    return *end == '\0';
}

static void lowerToken(char* token) {
    for (char* p = token; *p; ++p) {
        *p = static_cast<char>(tolower(*p));
    }
}

static void setDefaultPersistentConfig() {
    encoderCal.magic = kEncoderCalMagic;
    encoderCal.version = kEncoderCalVersion;
    encoderCal.sizeBytes = sizeof(EncoderCalibrationStorage);
    for (uint8_t i = 0; i < kMotorCount; ++i) {
        encoderCal.encoderConfigured[i] = (i == 0) ? 1 : 0;
        encoderCal.muxChannel[i] = i;
        encoderCal.zeroOffsetDeg[i] = 0.0f;
        encoderCal.positionPidP[i] = static_cast<float>(kCfg.positionGains.p);
        encoderCal.positionPidI[i] = static_cast<float>(kCfg.positionGains.i);
        encoderCal.positionPidD[i] = static_cast<float>(kCfg.positionGains.d);
    }
    encoderCal.muxChannel[0] = 0;
}

static void savePersistentConfig() {
    encoderCal.magic = kEncoderCalMagic;
    encoderCal.version = kEncoderCalVersion;
    encoderCal.sizeBytes = sizeof(EncoderCalibrationStorage);
    for (uint8_t i = 0; i < kMotorCount; ++i) {
        const MotorController::PidGains gains = motors[i]->getPositionGains();
        encoderCal.encoderConfigured[i] = encoders[i].configured ? 1 : 0;
        encoderCal.muxChannel[i] = encoders[i].muxChannel;
        encoderCal.zeroOffsetDeg[i] = static_cast<float>(encoders[i].zeroOffsetDeg);
        encoderCal.positionPidP[i] = static_cast<float>(gains.p);
        encoderCal.positionPidI[i] = static_cast<float>(gains.i);
        encoderCal.positionPidD[i] = static_cast<float>(gains.d);
    }
    EEPROM.put(0, encoderCal);
    EEPROM.commit();
}

static void applyPersistentConfig() {
    for (uint8_t i = 0; i < kMotorCount; ++i) {
        encoders[i].zeroOffsetDeg = AbsoluteEncoderMux::normalize0To360(encoderCal.zeroOffsetDeg[i]);
        encoders[i].muxChannel = encoderCal.muxChannel[i] <= 7 ? encoderCal.muxChannel[i] : i;
        encoders[i].configured = encoderCal.encoderConfigured[i] != 0 && encoders[i].muxChannel <= 7;
        encoders[i].valid = false;
        encoders[i].consecutiveFailures = 0;
        encoders[i].lastError = 0;

        motors[i]->setPositionGains(
            encoderCal.positionPidP[i],
            encoderCal.positionPidI[i],
            encoderCal.positionPidD[i]
        );
        motors[i]->enableAbsoluteFeedback(encoders[i].configured);
    }
}

static void loadPersistentConfig() {
    EEPROM.begin(sizeof(EncoderCalibrationStorage));
    EEPROM.get(0, encoderCal);

    if (encoderCal.magic != kEncoderCalMagic
        || encoderCal.version != kEncoderCalVersion
        || encoderCal.sizeBytes != sizeof(EncoderCalibrationStorage)) {
        setDefaultPersistentConfig();
        EEPROM.put(0, encoderCal);
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
    savePersistentConfig();
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
    savePersistentConfig();
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
    savePersistentConfig();
    return true;
}

static const char* modeName(MotorController::Mode mode) {
    switch (mode) {
        case MotorController::Mode::Velocity: return "SPD";
        case MotorController::Mode::Position: return "POS";
        case MotorController::Mode::Current: return "CUR";
        case MotorController::Mode::Stopped:
        default: return "STOP";
    }
}

static void printHelp() {
    Serial.println("# Commands:");
    Serial.println("# v id rpm          -> velocity target in output rpm");
    Serial.println("# p id deg          -> position target in degrees");
    Serial.println("# i id mA           -> direct current command for low-level tests");
    Serial.println("# l id rpm          -> position loop velocity limit");
    Serial.println("# k id 1|2          -> motor gearing/type: 1=M3508/C620, 2=M2006/C610");
    Serial.println("# e id chan         -> attach AS5600 on PCA9548A mux channel");
    Serial.println("# ed id             -> disable external encoder feedback");
    Serial.println("# z id              -> save current AS5600 angle as zero degrees");
    Serial.println("# pid id p i d      -> tune position PID gains live, in degree units");
    Serial.println("# t                 -> toggle telemetry");
    Serial.println("# m                 -> toggle human/csv telemetry");
    Serial.println("# s                 -> stop all motors");
}

static void doCmd(char* line) {
    Serial.print("# RECV: ["); Serial.print(line); Serial.println("]");
    char* tok = strtok(line, " \t");
    if (!tok) return;
    lowerToken(tok);

    if (strcmp(tok, "h") == 0 || strcmp(tok, "?") == 0) {
        printHelp();
        return;
    }

    if (strcmp(tok, "t") == 0) {
        teleEnabled = !teleEnabled;
        Serial.println(teleEnabled ? "# OK TELE ON" : "# OK TELE OFF");
        return;
    }

    if (strcmp(tok, "m") == 0) {
        humanMode = !humanMode;
        Serial.println(humanMode ? "# OK HUMAN TELE" : "# OK CSV TELE");
        return;
    }

    if (strcmp(tok, "s") == 0) {
        for (uint8_t i = 0; i < kMotorCount; ++i) motors[i]->stop();
        Serial.println("# OK STOP ALL");
        return;
    }

    if (strcmp(tok, "pid") == 0) {
        char* idT = strtok(nullptr, " \t");
        char* pT = strtok(nullptr, " \t");
        char* iT = strtok(nullptr, " \t");
        char* dT = strtok(nullptr, " \t");
        if (!idT || !pT || !iT || !dT) {
            Serial.println("# ERR USAGE pid id p i d");
            return;
        }

        const int id = atoi(idT);
        double p = 0.0, i = 0.0, d = 0.0;
        if (id < 1 || id > kMotorCount || !parseD(pT, p) || !parseD(iT, i) || !parseD(dT, d)) {
            Serial.println("# ERR BAD PID ARGS");
            return;
        }

        motors[id - 1]->setPositionGains(p, i, d);
        savePersistentConfig();
        Serial.print("# OK ID "); Serial.print(id);
        Serial.print(" PID="); Serial.print(p, 6);
        Serial.print(","); Serial.print(i, 6);
        Serial.print(","); Serial.println(d, 6);
        return;
    }

    char* idT = strtok(nullptr, " \t");
    if (!idT) {
        Serial.println("# ERR MISSING ID");
        return;
    }

    const int id = atoi(idT);
    if (id < 1 || id > kMotorCount) {
        Serial.println("# ERR BAD ID");
        return;
    }

    if (strcmp(tok, "z") == 0) {
        if (setEncoderZero(static_cast<uint8_t>(id))) {
            Serial.print("# OK ID "); Serial.print(id); Serial.println(" ZERO SAVED");
        } else {
            Serial.print("# ERR ID "); Serial.print(id); Serial.println(" ENCODER NOT VALID");
        }
        return;
    }

    if (strcmp(tok, "ed") == 0) {
        disableEncoder(static_cast<uint8_t>(id));
        Serial.print("# OK ID "); Serial.print(id); Serial.println(" EXT_ENC OFF");
        return;
    }

    char* valT = strtok(nullptr, " \t");
    if (!valT) {
        Serial.println("# ERR MISSING VALUE");
        return;
    }

    double val = 0.0;
    if (!parseD(valT, val)) {
        Serial.println("# ERR BAD VALUE");
        return;
    }

    MotorController* motor = motors[id - 1];
    if (strcmp(tok, "v") == 0) {
        motor->setTargetVelocity(val);
        Serial.print("# OK ID "); Serial.print(id); Serial.print(" V="); Serial.println(val);
    } else if (strcmp(tok, "p") == 0) {
        motor->setTargetPositionDeg(val);
        Serial.print("# OK ID "); Serial.print(id); Serial.print(" P_DEG="); Serial.println(val);
    } else if (strcmp(tok, "i") == 0) {
        motor->setTargetCurrent(val);
        Serial.print("# OK ID "); Serial.print(id); Serial.print(" I="); Serial.println(val);
    } else if (strcmp(tok, "l") == 0) {
        motor->setLimitV(val);
        Serial.print("# OK ID "); Serial.print(id); Serial.print(" L="); Serial.println(val);
    } else if (strcmp(tok, "k") == 0) {
        motor->setMotorType(static_cast<int>(val) == 1);
        Serial.print("# OK ID "); Serial.print(id); Serial.print(" T=");
        Serial.println(static_cast<int>(val) == 1 ? "3508" : "2006");
    } else if (strcmp(tok, "e") == 0) {
        const int channel = static_cast<int>(val);
        if (channel < 0 || channel > 7 || !configureEncoder(static_cast<uint8_t>(id), static_cast<uint8_t>(channel))) {
            Serial.println("# ERR BAD ENCODER CHANNEL");
            return;
        }
        Serial.print("# OK ID "); Serial.print(id); Serial.print(" EXT_ENC CHAN "); Serial.println(channel);
    } else {
        Serial.println("# ERR UNKNOWN OP");
    }
}

static void checkSerial() {
    while (Serial.available()) {
        const char c = static_cast<char>(Serial.read());
        if (c == '\n' || c == '\r') {
            if (cmdLen > 0) {
                cmdBuf[cmdLen] = '\0';
                doCmd(cmdBuf);
                cmdLen = 0;
            }
        } else if (cmdLen < sizeof(cmdBuf) - 1) {
            cmdBuf[cmdLen++] = c;
        }
    }
}

static void pad(double v, int width, int precision) {
    String s = String(v, precision);
    Serial.print(s);
    for (int i = 0; i < width - static_cast<int>(s.length()); ++i) Serial.print(' ');
}

static void printTelemetry(uint32_t now) {
    if (humanMode) {
        Serial.println("\nID | TYPE | MODE | TGT_V | ACT_V | LIM_V | TGT_D | POS_D | TMP | CURR | FB      | ENC");
        for (uint8_t i = 0; i < kMotorCount; ++i) {
            if (!motors[i]->isActive() && !encoders[i].configured) {
                continue;
            }

            Serial.print(i + 1); Serial.print("  | ");
            Serial.print(strcmp(motors[i]->getMotorType(), "M3508") == 0 ? "3508" : "2006"); Serial.print(" | ");
            Serial.print(modeName(motors[i]->getMode())); Serial.print(" | ");
            pad(motors[i]->getTargetV(), 5, 1); Serial.print(" | ");
            pad(motors[i]->getOutputVelocity(), 5, 1); Serial.print(" | ");
            pad(motors[i]->getLimitV(), 5, 0); Serial.print(" | ");
            pad(motors[i]->getTargetPDeg(), 5, 2); Serial.print(" | ");
            pad(motors[i]->getOutputPositionDeg(), 5, 2); Serial.print(" | ");
            Serial.print(motors[i]->getTemp()); Serial.print("C | ");
            pad(motors[i]->getCurr(), 4, 0); Serial.print(" | ");
            Serial.print(motors[i]->isUsingExternal() ? "AS5600 " : "CAN_ENC"); Serial.print(" | ");
            if (encoders[i].configured) {
                Serial.print(encoders[i].valid ? "OK ch" : "BAD ch");
                Serial.print(encoders[i].muxChannel);
                if (!encoders[i].valid) {
                    Serial.print(" err");
                    Serial.print(encoders[i].lastError);
                }
            } else {
                Serial.print("--");
            }
            Serial.println();
        }
        return;
    }

    for (uint8_t i = 0; i < kMotorCount; ++i) {
        if (!motors[i]->isActive() && !encoders[i].configured) {
            continue;
        }

        Serial.print(now); Serial.print(',');
        Serial.print(i + 1); Serial.print(',');
        Serial.print(motors[i]->getPositionFeedbackDeg(), 6); Serial.print(',');
        Serial.print(motors[i]->getOutputVelocity(), 3); Serial.print(',');
        Serial.print(motors[i]->getTemp()); Serial.print(',');
        Serial.print(motors[i]->getCurr()); Serial.print(',');
        Serial.print(motors[i]->isUsingExternal() ? "AS5600" : "CAN_ENC"); Serial.print(',');
        Serial.print(encoders[i].configured ? encoders[i].muxChannel : 255); Serial.print(',');
        Serial.print(encoders[i].valid ? 1 : 0); Serial.print(',');
        Serial.println(encoders[i].lastError);
    }
}

void setup() {
    Serial.begin(115200);

    encoderMux.begin();
    loadPersistentConfig();

    bus.begin();

    c1.setMotorType(true);   // Motor 1: M3508/C620
    c2.setMotorType(false);  // Motor 2: M2006/C610

    Serial.println("# SYSTEM READY. Type 'h' for help.");
    Serial.println("# AS5600 mux on Wire1 SDA=6 SCL=7 at 100kHz. Encoder map is loaded from EEPROM.");
}

void loop() {
    bus.update();
    pollOneConfiguredEncoder();
    checkSerial();

    for (uint8_t i = 0; i < kMotorCount; ++i) {
        motors[i]->update();
    }

    const uint32_t now = millis();
    if (teleEnabled && (now - lastTeleMs >= kTelemetryIntervalMs)) {
        printTelemetry(now);
        lastTeleMs = now;
    }

    loopCtrl.update();
}
