from __future__ import annotations

import time
from dataclasses import dataclass

from . import protocol


@dataclass
class SerialClient:
    port: str
    baud: int = 921600
    timeout: float = 1.0

    def __post_init__(self) -> None:
        import serial  # Imported here so documentation builds do not require pyserial.

        self._serial_mod = serial
        self._serial = None
        self._sequence = 1

    def __enter__(self) -> "SerialClient":
        self.open()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def open(self) -> None:
        if self._serial is None:
            try:
                self._serial = self._serial_mod.Serial(
                    port=self.port,
                    baudrate=self.baud,
                    timeout=self.timeout,
                    write_timeout=self.timeout,
                )
                self._serial.reset_input_buffer()
                self._serial.reset_output_buffer()
            except Exception as exc:
                raise RuntimeError(
                    f"could not open {self.port}. Make sure the XIAO is attached to Docker, "
                    "Arduino Serial Monitor is closed, and the port name is correct."
                ) from exc

    def close(self) -> None:
        if self._serial is not None:
            self._serial.close()
            self._serial = None

    def next_sequence(self) -> int:
        sequence = self._sequence
        self._sequence = (self._sequence + 1) & 0xFFFFFFFF
        if self._sequence == 0:
            self._sequence = 1
        return sequence

    def send_raw_packet(self, packet: bytes) -> None:
        if self._serial is None:
            raise RuntimeError("serial port is not open")
        frame = protocol.cobs_encode(packet) + b"\x00"
        self._serial.write(frame)
        self._serial.flush()

    def read_packet(self, timeout: float | None = None) -> bytes:
        if self._serial is None:
            raise RuntimeError("serial port is not open")
        deadline = time.monotonic() + (self.timeout if timeout is None else timeout)
        frame = bytearray()
        while time.monotonic() < deadline:
            byte = self._serial.read(1)
            if not byte:
                continue
            if byte == b"\x00":
                if frame:
                    return protocol.cobs_decode(bytes(frame))
                continue
            frame.extend(byte)
        raise TimeoutError(f"timed out waiting for a packet on {self.port}")

    def send_config_and_wait_ack(self, *, command: int, motor_id: int, **kwargs) -> protocol.ConfigAck:
        sequence = self.next_sequence()
        packet = protocol.make_config_packet(
            sequence=sequence,
            command=command,
            motor_id=motor_id,
            **kwargs,
        )
        self.send_raw_packet(packet)
        while True:
            response = self.read_packet()
            if not protocol.validate(response):
                continue
            packet_type = response[0]
            if packet_type != protocol.PACKET_CONFIG_ACK:
                continue
            ack = protocol.parse_config_ack(response)
            if ack.sequence == sequence:
                return ack

    def request_telemetry(self) -> protocol.Telemetry:
        sequence = self.next_sequence()
        packet = protocol.make_telemetry_request(sequence=sequence, host_time_ms=int(time.monotonic() * 1000))
        self.send_raw_packet(packet)
        while True:
            response = self.read_packet()
            if not protocol.validate(response):
                continue
            if response[0] != protocol.PACKET_TELEMETRY:
                continue
            telemetry = protocol.parse_telemetry(response)
            if telemetry.sequence == sequence:
                return telemetry

    def send_halt(self) -> None:
        self.send_raw_packet(protocol.make_halt_packet(self.next_sequence()))
