"""Python copy of the RP2350 binary protocol.

The C++ firmware and hardware-interface headers remain the source of truth for
the packet layout. This module intentionally mirrors those packed structs so
small calibration tools can talk directly to the firmware without the old text
Serial shell.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import Iterable, Optional


PROTOCOL_VERSION = 1
MAX_MOTORS = 8

PACKET_COMMAND = 1
PACKET_TELEMETRY = 2
PACKET_CONFIG = 3
PACKET_CONFIG_ACK = 4
PACKET_TELEMETRY_REQUEST = 5
PACKET_HALT = 6

MODE_DISABLED = 0
MODE_POSITION = 1
MODE_VELOCITY = 2
MODE_CURRENT = 3
MODE_STOP = 4

CONFIG_NONE = 0
CONFIG_ENABLE_ENCODER = 1
CONFIG_DISABLE_ENCODER = 2
CONFIG_ZERO_CURRENT_POSITION = 3
CONFIG_SET_POSITION_PID = 4
CONFIG_SET_MOTOR_TYPE = 5
CONFIG_SET_VELOCITY_LIMIT = 6
CONFIG_SAVE_CONFIG = 7

STATUS_OK = 0
STATUS_BAD_MOTOR_ID = 1
STATUS_BAD_CHANNEL = 2
STATUS_ENCODER_INVALID = 3
STATUS_BAD_COMMAND = 4

HEADER = struct.Struct("<BBHIH")
COMMAND = struct.Struct("<BBHIHIB8B8f8f8f")
TELEMETRY_REQUEST = struct.Struct("<BBHIHI")
CONFIG = struct.Struct("<BBHIHBBBBfffff")
CONFIG_ACK = struct.Struct("<BBHIHBBBB")
TELEMETRY_PREFIX = struct.Struct("<BBHIHIIBB")

STATUS_TEXT = {
    STATUS_OK: "ok",
    STATUS_BAD_MOTOR_ID: "bad motor id",
    STATUS_BAD_CHANNEL: "bad encoder channel",
    STATUS_ENCODER_INVALID: "encoder invalid",
    STATUS_BAD_COMMAND: "bad command",
}

CONFIG_TEXT = {
    CONFIG_ENABLE_ENCODER: "enable encoder",
    CONFIG_DISABLE_ENCODER: "disable encoder",
    CONFIG_ZERO_CURRENT_POSITION: "zero current position",
    CONFIG_SET_POSITION_PID: "set position pid",
    CONFIG_SET_MOTOR_TYPE: "set motor type",
    CONFIG_SET_VELOCITY_LIMIT: "set velocity limit",
    CONFIG_SAVE_CONFIG: "save config",
}


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def cobs_encode(data: bytes) -> bytes:
    output = bytearray()
    code_index = 0
    output.append(0)
    code = 1

    for value in data:
        if value == 0:
            output[code_index] = code
            code_index = len(output)
            output.append(0)
            code = 1
        else:
            output.append(value)
            code += 1
            if code == 0xFF:
                output[code_index] = code
                code_index = len(output)
                output.append(0)
                code = 1

    output[code_index] = code
    return bytes(output)


def cobs_decode(data: bytes) -> bytes:
    output = bytearray()
    index = 0
    length = len(data)

    while index < length:
        code = data[index]
        if code == 0:
            raise ValueError("zero byte inside COBS frame")
        index += 1

        for _ in range(code - 1):
            if index >= length:
                raise ValueError("truncated COBS frame")
            output.append(data[index])
            index += 1

        if code != 0xFF and index < length:
            output.append(0)

    return bytes(output)


def finalize(packet_type: int, sequence: int, payload_format: struct.Struct, payload_values: Iterable) -> bytes:
    payload_size = payload_format.size - HEADER.size
    values = [packet_type, PROTOCOL_VERSION, payload_size, sequence, 0]
    values.extend(payload_values)
    packet = bytearray(payload_format.pack(*values))
    crc = crc16_ccitt(packet)
    HEADER.pack_into(packet, 0, packet_type, PROTOCOL_VERSION, payload_size, sequence, crc)
    return bytes(packet)


def validate(packet: bytes, expected_type: Optional[int] = None) -> bool:
    if len(packet) < HEADER.size:
        return False
    packet_type, version, payload_size, _sequence, crc = HEADER.unpack_from(packet)
    if expected_type is not None and packet_type != expected_type:
        return False
    if version != PROTOCOL_VERSION:
        return False
    if payload_size != len(packet) - HEADER.size:
        return False
    copy = bytearray(packet)
    HEADER.pack_into(copy, 0, packet_type, version, payload_size, _sequence, 0)
    return crc16_ccitt(copy) == crc


def make_config_packet(
    *,
    sequence: int,
    command: int,
    motor_id: int,
    encoder_channel: int = 0,
    motor_type: int = 1,
    zero_offset_deg: float = 0.0,
    position_pid_p: float = 0.0,
    position_pid_i: float = 0.0,
    position_pid_d: float = 0.0,
    velocity_limit_rpm: float = 0.0,
) -> bytes:
    return finalize(
        PACKET_CONFIG,
        sequence,
        CONFIG,
        [
            command,
            motor_id,
            encoder_channel,
            motor_type,
            zero_offset_deg,
            position_pid_p,
            position_pid_i,
            position_pid_d,
            velocity_limit_rpm,
        ],
    )


def make_telemetry_request(sequence: int, host_time_ms: int) -> bytes:
    return finalize(PACKET_TELEMETRY_REQUEST, sequence, TELEMETRY_REQUEST, [host_time_ms & 0xFFFFFFFF])


def make_halt_packet(sequence: int) -> bytes:
    return finalize(PACKET_HALT, sequence, HEADER, [])


@dataclass
class ConfigAck:
    sequence: int
    command: int
    motor_id: int
    status: int


def parse_config_ack(packet: bytes) -> ConfigAck:
    if len(packet) != CONFIG_ACK.size or not validate(packet, PACKET_CONFIG_ACK):
        raise ValueError("invalid ConfigAckPacket")
    packet_type, version, payload_size, sequence, crc, command, motor_id, status, _reserved = CONFIG_ACK.unpack(packet)
    del packet_type, version, payload_size, crc
    return ConfigAck(sequence=sequence, command=command, motor_id=motor_id, status=status)


@dataclass
class MotorTelemetry:
    active: int
    mode: int
    encoder_configured: int
    encoder_valid: int
    encoder_channel: int
    i2c_error: int
    position_deg: float
    velocity_rpm: float
    current_ma: float
    temperature_c: float


@dataclass
class Telemetry:
    sequence: int
    mcu_time_ms: int
    last_command_age_ms: int
    motor_count: int
    watchdog_tripped: int
    motors: list[MotorTelemetry]


def parse_telemetry(packet: bytes) -> Telemetry:
    if not validate(packet, PACKET_TELEMETRY):
        raise ValueError("invalid TelemetryPacket")

    packet_type, version, payload_size, sequence, crc, mcu_time_ms, age_ms, motor_count, watchdog = TELEMETRY_PREFIX.unpack_from(packet)
    del packet_type, version, payload_size, crc

    offset = TELEMETRY_PREFIX.size
    active = list(packet[offset : offset + MAX_MOTORS])
    offset += MAX_MOTORS
    modes = list(packet[offset : offset + MAX_MOTORS])
    offset += MAX_MOTORS
    enc_cfg = list(packet[offset : offset + MAX_MOTORS])
    offset += MAX_MOTORS
    enc_valid = list(packet[offset : offset + MAX_MOTORS])
    offset += MAX_MOTORS
    enc_channel = list(packet[offset : offset + MAX_MOTORS])
    offset += MAX_MOTORS
    i2c_error = list(packet[offset : offset + MAX_MOTORS])
    offset += MAX_MOTORS

    floats = struct.unpack_from("<32f", packet, offset)
    positions = floats[0:8]
    velocities = floats[8:16]
    currents = floats[16:24]
    temps = floats[24:32]

    motors = []
    for i in range(MAX_MOTORS):
        motors.append(
            MotorTelemetry(
                active=active[i],
                mode=modes[i],
                encoder_configured=enc_cfg[i],
                encoder_valid=enc_valid[i],
                encoder_channel=enc_channel[i],
                i2c_error=i2c_error[i],
                position_deg=positions[i],
                velocity_rpm=velocities[i],
                current_ma=currents[i],
                temperature_c=temps[i],
            )
        )

    return Telemetry(
        sequence=sequence,
        mcu_time_ms=mcu_time_ms,
        last_command_age_ms=age_ms,
        motor_count=motor_count,
        watchdog_tripped=watchdog,
        motors=motors,
    )
