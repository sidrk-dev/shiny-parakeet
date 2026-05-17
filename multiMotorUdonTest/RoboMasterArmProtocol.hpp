#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

namespace RoboMasterArmProtocol
{
static constexpr uint8_t kProtocolVersion = 1;
static constexpr uint8_t kMaxMotors = 8;
static constexpr size_t kMaxPacketBytes = 256;
static constexpr size_t kMaxEncodedBytes = kMaxPacketBytes + (kMaxPacketBytes / 254) + 2;

enum class PacketType : uint8_t {
    Command = 1,
    Telemetry = 2,
    Config = 3,
    ConfigAck = 4,
    TelemetryRequest = 5,
    Halt = 6,
};

enum class MotorMode : uint8_t {
    Disabled = 0,
    Position = 1,
    Velocity = 2,
    Current = 3,
    Stop = 4,
};

enum class ConfigCommand : uint8_t {
    None = 0,
    EnableEncoder = 1,
    DisableEncoder = 2,
    ZeroCurrentPosition = 3,
    SetPositionPid = 4,
    SetMotorType = 5,
    SetVelocityLimit = 6,
    SaveConfig = 7,
};

enum class ConfigStatus : uint8_t {
    Ok = 0,
    BadMotorId = 1,
    BadChannel = 2,
    EncoderInvalid = 3,
    BadCommand = 4,
};

#pragma pack(push, 1)
struct PacketHeader {
    uint8_t type;
    uint8_t version;
    uint16_t payload_size;
    uint32_t sequence;
    uint16_t crc16;
};

struct CommandPacket {
    PacketHeader header;
    uint32_t host_time_ms;
    uint8_t motor_count;
    uint8_t mode[kMaxMotors];
    float target_position_deg[kMaxMotors];
    float target_velocity_rpm[kMaxMotors];
    float target_current_ma[kMaxMotors];
};

struct TelemetryRequestPacket {
    PacketHeader header;
    uint32_t host_time_ms;
};

struct ConfigPacket {
    PacketHeader header;
    uint8_t command;
    uint8_t motor_id;
    uint8_t encoder_channel;
    uint8_t motor_type;
    float zero_offset_deg;
    float position_pid_p;
    float position_pid_i;
    float position_pid_d;
    float velocity_limit_rpm;
};

struct TelemetryPacket {
    PacketHeader header;
    uint32_t mcu_time_ms;
    uint32_t last_command_age_ms;
    uint8_t motor_count;
    uint8_t watchdog_tripped;
    uint8_t active[kMaxMotors];
    uint8_t mode[kMaxMotors];
    uint8_t encoder_configured[kMaxMotors];
    uint8_t encoder_valid[kMaxMotors];
    uint8_t encoder_channel[kMaxMotors];
    uint8_t i2c_error[kMaxMotors];
    float position_deg[kMaxMotors];
    float velocity_rpm[kMaxMotors];
    float current_ma[kMaxMotors];
    float temperature_c[kMaxMotors];
};

struct ConfigAckPacket {
    PacketHeader header;
    uint8_t command;
    uint8_t motor_id;
    uint8_t status;
    uint8_t reserved;
};
#pragma pack(pop)

static_assert(sizeof(CommandPacket) <= kMaxPacketBytes, "CommandPacket exceeds fixed protocol buffer");
static_assert(sizeof(TelemetryPacket) <= kMaxPacketBytes, "TelemetryPacket exceeds fixed protocol buffer");
static_assert(sizeof(ConfigPacket) <= kMaxPacketBytes, "ConfigPacket exceeds fixed protocol buffer");
static_assert(sizeof(ConfigAckPacket) <= kMaxPacketBytes, "ConfigAckPacket exceeds fixed protocol buffer");
static_assert(sizeof(TelemetryRequestPacket) <= kMaxPacketBytes, "TelemetryRequestPacket exceeds fixed protocol buffer");

inline uint16_t crc16Ccitt(const uint8_t* data, size_t length)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; ++i) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (uint8_t bit = 0; bit < 8; ++bit) {
            if ((crc & 0x8000U) != 0) {
                crc = static_cast<uint16_t>((crc << 1) ^ 0x1021U);
            } else {
                crc = static_cast<uint16_t>(crc << 1);
            }
        }
    }
    return crc;
}

inline size_t expectedPacketSize(PacketType type)
{
    switch (type) {
        case PacketType::Command: return sizeof(CommandPacket);
        case PacketType::Telemetry: return sizeof(TelemetryPacket);
        case PacketType::Config: return sizeof(ConfigPacket);
        case PacketType::ConfigAck: return sizeof(ConfigAckPacket);
        case PacketType::TelemetryRequest: return sizeof(TelemetryRequestPacket);
        case PacketType::Halt: return sizeof(PacketHeader);
        default: return 0;
    }
}

template <typename PacketT>
inline void finalizePacket(PacketT& packet, PacketType type, uint32_t sequence)
{
    packet.header.type = static_cast<uint8_t>(type);
    packet.header.version = kProtocolVersion;
    packet.header.payload_size = static_cast<uint16_t>(sizeof(PacketT) - sizeof(PacketHeader));
    packet.header.sequence = sequence;
    packet.header.crc16 = 0;
    packet.header.crc16 = crc16Ccitt(reinterpret_cast<const uint8_t*>(&packet), sizeof(PacketT));
}

template <typename PacketT>
inline bool validatePacket(const PacketT& packet, PacketType type)
{
    if (packet.header.type != static_cast<uint8_t>(type)) return false;
    if (packet.header.version != kProtocolVersion) return false;
    if (packet.header.payload_size != sizeof(PacketT) - sizeof(PacketHeader)) return false;

    PacketT copy;
    memcpy(&copy, &packet, sizeof(PacketT));
    const uint16_t expected_crc = copy.header.crc16;
    copy.header.crc16 = 0;
    return crc16Ccitt(reinterpret_cast<const uint8_t*>(&copy), sizeof(PacketT)) == expected_crc;
}

inline size_t cobsEncode(const uint8_t* input, size_t length, uint8_t* output, size_t output_capacity)
{
    if (output_capacity < length + (length / 254U) + 1U) {
        return 0;
    }

    const uint8_t* end = input + length;
    uint8_t* start = output;
    uint8_t* code_ptr = output++;
    uint8_t code = 1;

    while (input < end) {
        if (*input == 0) {
            *code_ptr = code;
            code_ptr = output++;
            code = 1;
            input++;
        } else {
            *output++ = *input++;
            code++;
            if (code == 0xFF) {
                *code_ptr = code;
                code_ptr = output++;
                code = 1;
            }
        }
    }

    *code_ptr = code;
    return static_cast<size_t>(output - start);
}

inline size_t cobsDecode(const uint8_t* input, size_t length, uint8_t* output, size_t output_capacity)
{
    size_t read_index = 0;
    size_t write_index = 0;

    while (read_index < length) {
        const uint8_t code = input[read_index++];
        if (code == 0) {
            return 0;
        }

        for (uint8_t i = 1; i < code; ++i) {
            if (read_index >= length || write_index >= output_capacity) {
                return 0;
            }
            output[write_index++] = input[read_index++];
        }

        if (code != 0xFF && read_index < length) {
            if (write_index >= output_capacity) {
                return 0;
            }
            output[write_index++] = 0;
        }
    }

    return write_index;
}
} // namespace RoboMasterArmProtocol
