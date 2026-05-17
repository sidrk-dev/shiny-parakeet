# multiMotorUdonTest Firmware

Production-oriented RP2350 firmware for the RoboMaster arm test platform.

This sketch controls DJI RoboMaster motors over CAN with UdonLibrary and reads
AS5600 absolute encoders through a PCA9548A I2C mux. The firmware now exposes a
COBS-framed binary protocol for `ros2_control`; it no longer uses the old
human-readable Serial command shell.

## Hardware Target

- MCU: Seeed Studio XIAO RP2350
- CAN: MCP2515-compatible SPI CAN adapter
- Motors: DJI RoboMaster M3508/C620 and M2006/C610
- Encoders: AS5600 absolute encoders
- Encoder mux: PCA9548A

## Pinout

### CAN SPI

| Signal | Pin |
|---|---:|
| CS | `D3` |
| MOSI | `D10` |
| MISO | `D9` |
| SCK | `D8` |

### Encoder I2C

| Signal | Pin |
|---|---:|
| SDA | `6` |
| SCL | `7` |
| Bus | `Wire1` |
| Clock | `100 kHz` |

The AS5600 read path intentionally uses a standard stop-start I2C transaction,
not a repeated start. This matches the known-good `encoderTest` sketch.

## Firmware Files

- `multiMotorUdonTest.ino`: main loop, CAN, EEPROM, binary serial protocol,
  watchdog, packet handlers.
- `MotorController.hpp`: per-motor control wrapper. This still owns position,
  velocity, current, PID state, and safe feedback transitions.
- `AbsoluteEncoder.hpp`: PCA9548A + AS5600 I2C reader.
- `RoboMasterArmProtocol.hpp`: shared packet definitions, CRC16, and COBS
  encode/decode helpers for the MCU side.

## Control Loop

The main loop is timed with:

```cpp
Udon::LoopCycleController loopCtrl{1000};
```

That means the target control period is 1 ms.

Each loop:

1. Updates the CAN bus.
2. Polls one configured encoder for regular control freshness.
3. Processes any complete COBS Serial frames.
4. Enforces the 100 ms host watchdog.
5. Updates all `MotorController` instances.
6. Waits for the next 1 ms cycle.

When the ROS host requests telemetry, the firmware snapshots all configured
encoders back-to-back before building the telemetry packet. This reduces
kinematic skew between joints in a telemetry sample.

## Serial Protocol

Baud rate:

```text
921600
```

Framing:

```text
COBS(payload_with_crc) + 0x00 delimiter
```

Every packet contains:

- packet type
- protocol version
- payload size
- sequence number
- CRC16-CCITT

The packet definitions live in:

```text
RoboMasterArmProtocol.hpp
```

The ROS copy lives in:

```text
ros2_ws/src/robomaster_arm_hw/include/robomaster_arm_hw/protocol.hpp
```

Keep these two headers byte-compatible.

## Packet Types

### `CommandPacket`

Direction:

```text
ROS host -> MCU
```

Purpose:

Sets motor targets for up to 8 motors.

Per motor:

- mode
- target position in degrees
- target velocity in output RPM
- target current in milliamps

The ROS hardware interface currently sends position commands.

### `TelemetryRequestPacket`

Direction:

```text
ROS host -> MCU
```

Purpose:

Requests a telemetry snapshot.

The firmware responds with `TelemetryPacket`.

### `TelemetryPacket`

Direction:

```text
MCU -> ROS host
```

Purpose:

Reports state for up to 8 motors.

Per motor:

- active flag
- control mode
- encoder configured flag
- encoder valid flag
- encoder channel
- last I2C error
- position in degrees
- velocity in output RPM
- current in milliamps
- temperature in Celsius

### `ConfigPacket`

Direction:

```text
ROS host -> MCU
```

Purpose:

Replaces the old text commands for calibration and tuning.

Supported config commands:

- enable encoder
- disable encoder
- zero current encoder position
- set position PID
- set motor type
- set position velocity limit
- save config

Successful config changes are persisted to EEPROM.

### `ConfigAckPacket`

Direction:

```text
MCU -> ROS host
```

Purpose:

Acknowledges a `ConfigPacket` and reports success or failure.

## Watchdog

The MCU requires a valid `CommandPacket` at least every:

```text
100 ms
```

If the host stops sending valid command packets:

1. `watchdogTripped` becomes true.
2. All `MotorController` instances are forced to stopped mode.
3. Motor current commands go to zero.

This is the main fail-safe for ROS crashes, unplugged USB, Docker disconnects,
or controller-manager failures.

## EEPROM Persistence

The firmware persists:

- encoder enabled/disabled state
- PCA9548A mux channel
- motor type
- encoder zero offset
- position PID gains
- position velocity limit

EEPROM is loaded at boot. If the stored structure version does not match the
firmware, defaults are written.

Default config:

- Motor 1 encoder enabled on mux channel 0.
- Motor 1 type M3508/C620.
- Motor 2 type M2006/C610.
- Other motors default M3508/C620 expansion slots.

## Units

Firmware protocol units:

| Value | Unit |
|---|---|
| position | degrees |
| velocity | output RPM |
| current | milliamps |
| temperature | Celsius |

ROS hardware interface units:

| Value | Unit |
|---|---|
| position | radians |
| velocity | radians per second |
| effort | amps, currently motor current |

The ROS hardware interface converts between the two.

## Build

Compile only:

```powershell
.\compile.cmd
```

or:

```powershell
.\compile.ps1
```

These scripts do not upload and do not open a Serial monitor.

## Safety Bring-Up

Before connecting ROS controllers:

1. Verify CAN wiring and motor IDs.
2. Verify encoder mux channels.
3. Verify encoder direction against motor direction.
4. Keep position PID gains low.
5. Keep the arm supported.
6. Test watchdog behavior with the motors unloaded if possible.

The current firmware does not yet implement per-joint soft limits, encoder
direction inversion, or encoder-to-joint gear scaling. Those should be added
before high-power arm testing.
