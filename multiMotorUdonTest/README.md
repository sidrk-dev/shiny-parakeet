# multiMotorUdonTest

Beginner-friendly test firmware for controlling DJI RoboMaster motors from an
Arduino-compatible microcontroller while reading AS5600 absolute encoders
through a PCA9548A I2C multiplexer.

This project is currently aimed at the Seeed Studio XIAO RP2350 and a
MCP2515-style SPI CAN adapter. The firmware is written to be useful as a
standalone motor test program now, while keeping the structure close to what a
future ROS 2 / MoveIt hardware interface will need later.

## What This Firmware Does

The firmware can:

- Control up to 8 RoboMaster ESCs on one CAN bus.
- Drive C620/M3508 and C610/M2006 motors through UdonLibrary motor classes.
- Run a fixed 1 kHz control loop using `Udon::LoopCycleController`.
- Accept Serial commands for velocity, position, current, tuning, zeroing, and
  telemetry.
- Read AS5600 absolute encoders through a PCA9548A mux on `Wire1`.
- Store encoder mapping, encoder zero offsets, and position PID gains in the
  XIAO RP2350 EEPROM emulation.
- Continue running without blocking on Serial.
- Stop motor current if absolute encoder feedback is configured but becomes
  invalid.

The important idea is that motor control and encoder reading are separated:

- `AbsoluteEncoder.hpp` only knows how to talk to the AS5600 through the mux.
- `MotorController.hpp` only knows how to control a motor using feedback.
- `multiMotorUdonTest.ino` connects everything together and handles Serial
  commands, telemetry, EEPROM, CAN, and I2C.

## Very Important Safety Notes

RoboMaster motors are strong. Treat every position command as capable of moving
the mechanism suddenly.

Before testing:

1. Keep the robot arm mechanically supported.
2. Keep hands, wires, tools, and loose parts away from the drivetrain.
3. Start with the motor unloaded or with the joint disconnected if possible.
4. Use small position moves first, such as `p 1 5`.
5. Keep the Serial command `s` ready. It stops all motors.
6. Start with low position PID gains.
7. Do not assume the encoder direction matches the motor direction.

If the motor moves the wrong way, do not increase PID gains. Stop and check the
encoder direction, gear relationship, motor type, and wiring first.

## Hardware Overview

This firmware expects four main pieces of hardware:

1. Microcontroller: Seeed Studio XIAO RP2350.
2. CAN adapter: MCP2515 or compatible SPI CAN board.
3. Motor ESCs: DJI RoboMaster C620 and/or C610.
4. Absolute encoders: AS5600 connected through a PCA9548A I2C mux.

The current default hardware assumption is:

- Motor 1 is a M3508 driven by a C620 ESC.
- Motor 2 is a M2006 driven by a C610 ESC.
- Motors 3 through 8 are available expansion slots.
- Motor 1 currently has an AS5600 connected to PCA9548A mux channel 0.

After EEPROM has been initialized, the encoder mapping is loaded from EEPROM,
not from a hard-coded setup line. This means commands such as `e`, `ed`, `z`,
and `pid` persist across power cycles.

## Wiring Summary

### CAN Adapter Pins

The CAN pins are defined in `multiMotorUdonTest.ino`.

| Signal | XIAO RP2350 pin |
|---|---:|
| CAN CS | `D3` |
| CAN MOSI | `D10` |
| CAN MISO | `D9` |
| CAN SCK | `D8` |

The code uses `spi0`.

### AS5600 / PCA9548A I2C Pins

The absolute encoder bus uses the same settings as the working
`encoderTest.ino` sketch.

| Signal | XIAO RP2350 pin |
|---|---:|
| I2C SDA | `6` |
| I2C SCL | `7` |
| I2C peripheral | `Wire1` |
| I2C clock | `100 kHz` |
| PCA9548A address | `0x70` |
| AS5600 address | `0x36` |

The AS5600 read uses a standard stop-start sequence. It does not use repeated
starts.

### PCA9548A Channel Meaning

The PCA9548A is an I2C switch. Every AS5600 has the same fixed address, so the
mux lets the firmware choose which encoder is currently connected.

Example:

| Motor | PCA9548A channel |
|---:|---:|
| Motor 1 | Channel 0 |
| Motor 2 | Channel 1 |
| Motor 3 | Channel 2 |

You configure this with the `e` command:

```text
e 1 0
e 2 1
e 3 2
```

## Software Files

### `multiMotorUdonTest.ino`

This is the main Arduino sketch. It:

- Creates the CAN bus.
- Creates the motor objects.
- Creates the `MotorController` wrappers.
- Initializes `Wire1` for the AS5600 mux.
- Loads persistent settings from EEPROM.
- Polls one configured encoder per loop.
- Runs all motor controllers.
- Parses Serial commands.
- Prints telemetry.

### `MotorController.hpp`

This class owns the control logic for one motor.

It supports these modes:

- `Stopped`
- `Velocity`
- `Position`
- `Current`

It also handles:

- Motor type gearing.
- Current limiting using `motor.getCurrentRange()`.
- Position PID.
- Velocity PID.
- Encoder feedback validity.
- Clean transitions between modes.
- Re-basing the position target when zeroing or changing feedback source.

### `AbsoluteEncoder.hpp`

This class reads AS5600 raw angle values through the PCA9548A mux.

It handles:

- Selecting mux channels.
- Reading AS5600 raw angle register `0x0C`.
- Converting raw counts to degrees.
- Normalizing angles to `0` through `360`.
- Basic I2C recovery by restarting the bus after repeated failures.

### `compile.cmd` and `compile.ps1`

These scripts compile the project with Arduino CLI for:

```text
rp2040:rp2040:seeed_xiao_rp2350
```

The compile script only builds the firmware. It does not upload and does not
open the Serial monitor.

## Position Units

The Serial interface uses degrees for position commands and telemetry.

Examples:

```text
p 1 90
p 1 180
p 1 0
```

Important: this firmware reports the AS5600 encoder angle in degrees. That
does not automatically mean the robot joint moved the same number of degrees.

The reported position is physically correct only if:

1. The AS5600 magnet is attached directly to the joint shaft, or
2. The encoder-to-joint gear ratio is exactly 1:1, or
3. The firmware has been extended to apply the encoder-to-joint gear ratio.

If the encoder is on a motor shaft, intermediate gear shaft, belt pulley, or
anything other than the final joint, then `90` encoder degrees may not equal
`90` joint degrees.

This is a common source of confusion during arm bring-up.

## Direction Matters

Position control only works correctly if the feedback direction matches the
motor command direction.

If the controller commands positive current and the measured position goes
negative, the control loop will fight itself. Symptoms include:

- Moving in a random-looking direction.
- Running away from the target.
- Oscillating immediately.
- Reporting a position change that does not match the physical joint motion.
- Moving when you expect it to hold.

This README describes the current firmware behavior, but the next likely
feature for a multi-joint arm is a persisted encoder direction setting per
motor, such as:

```text
edir 1 1
edir 1 -1
```

That feature is not implemented in the current code yet.

## EEPROM Persistence

The firmware stores configuration in EEPROM emulation on the XIAO RP2350.

Currently stored per motor:

- Whether external encoder feedback is enabled.
- PCA9548A mux channel.
- Encoder zero offset in degrees.
- Position PID P gain.
- Position PID I gain.
- Position PID D gain.

These commands write to EEPROM:

```text
e id channel
ed id
z id
pid id p i d
```

Examples:

```text
e 1 0
pid 1 2.0 0.0 0.02
z 1
```

After running those commands, rebooting the board should keep:

- Motor 1 using AS5600 feedback.
- Motor 1 assigned to mux channel 0.
- Motor 1 using the saved zero offset.
- Motor 1 using the saved position PID gains.

EEPROM writes are useful, but they are not infinite. Do not write PID values in
a fast loop. It is fine to tune manually through the Serial console.

## Serial Command Reference

Commands are text lines sent over Serial at `115200` baud.

Arguments are separated by spaces.

### Help

```text
h
?
```

Prints the command list.

### Stop All Motors

```text
s
```

Stops every motor by setting the control mode to stopped and commanding zero
current.

Use this whenever behavior looks wrong.

### Velocity Control

```text
v id rpm
```

Sets a motor output velocity target in RPM.

Example:

```text
v 1 100
```

Motor 1 attempts to spin at `100` output RPM.

To reverse direction:

```text
v 1 -100
```

### Position Control

```text
p id deg
```

Sets a position target in degrees.

Example:

```text
p 1 90
```

Motor 1 attempts to move to `90` degrees according to its current feedback
source.

If an encoder is enabled and valid, position feedback comes from the AS5600.

If an encoder is not enabled, position feedback comes from the RoboMaster motor
CAN encoder.

If an encoder is enabled but invalid, the motor commands zero current rather
than falling back to another position source.

### Direct Current Test

```text
i id mA
```

Commands current directly in milliamps. This is for low-level testing only.

Example:

```text
i 1 500
```

This bypasses the position and velocity outer commands. Use small values and be
careful.

### Velocity Limit for Position Mode

```text
l id rpm
```

Sets the output RPM limit used by position control.

Example:

```text
l 1 100
```

This limits the position PID output so it cannot request more than `100` output
RPM.

### Motor Type / Gear Ratio

```text
k id 1
k id 2
```

Use:

- `1` for M3508/C620.
- `2` for M2006/C610.

Examples:

```text
k 1 1
k 2 2
```

The firmware uses this to choose the motor gear ratio:

- M3508: `19:1`
- M2006: `36:1`

This affects output velocity and CAN encoder position calculations.

### Attach an AS5600 Encoder

```text
e id channel
```

Attaches a motor to an AS5600 on a PCA9548A mux channel.

Example:

```text
e 1 0
```

This means:

- Motor ID: `1`
- PCA9548A channel: `0`

This setting is saved to EEPROM.

### Disable an Encoder

```text
ed id
```

Disables external encoder feedback for a motor.

Example:

```text
ed 1
```

After this, position feedback comes from the RoboMaster motor CAN encoder
instead of the AS5600.

This setting is saved to EEPROM.

### Zero an Encoder

```text
z id
```

Saves the current AS5600 angle as `0` degrees for that motor.

Example:

```text
z 1
```

This setting is saved to EEPROM.

The firmware also rebases the active position target to the new zero point, so
zeroing should not command a sudden move.

If `z` fails, the encoder is probably not configured or not currently reading
valid data.

### Tune Position PID

```text
pid id p i d
```

Sets position PID gains for one motor.

Example:

```text
pid 1 2.0 0.0 0.02
```

This setting is saved to EEPROM.

The position PID currently works in degree units.

That means the proportional term is roughly:

```text
requested_output_rpm = P * position_error_degrees
```

before limiting.

Example:

If:

```text
P = 2.0
error = 10 degrees
```

then the position loop requests about:

```text
20 output RPM
```

The velocity PID then converts that requested velocity into motor current.

### Toggle Telemetry

```text
t
```

Turns telemetry on or off.

### Toggle Human / CSV Telemetry

```text
m
```

Switches between human-readable telemetry and CSV telemetry.

Human-readable telemetry is easier to read in a Serial monitor.

CSV telemetry is easier to copy into logs, spreadsheets, or plotting tools.

## Telemetry Fields

In human mode, telemetry looks like this:

```text
ID | TYPE | MODE | TGT_V | ACT_V | LIM_V | TGT_D | POS_D | TMP | CURR | FB | ENC
```

Meaning:

| Field | Meaning |
|---|---|
| `ID` | Motor ID from 1 to 8 |
| `TYPE` | `3508` or `2006` |
| `MODE` | `STOP`, `SPD`, `POS`, or `CUR` |
| `TGT_V` | Target output velocity in RPM |
| `ACT_V` | Measured output velocity in RPM |
| `LIM_V` | Position-mode output velocity limit |
| `TGT_D` | Target position in degrees |
| `POS_D` | Feedback position in degrees |
| `TMP` | ESC-reported motor temperature in Celsius |
| `CURR` | ESC-reported torque current in milliamps |
| `FB` | Feedback source, `AS5600` or `CAN_ENC` |
| `ENC` | Encoder status and mux channel |

In CSV mode, each active or configured motor prints:

```text
time_ms,motor_id,position_deg,velocity_rpm,temp_c,current_mA,feedback,channel,encoder_valid,last_i2c_error
```

## Recommended First Bring-Up Procedure

Use this sequence when testing a motor and encoder for the first time.

### 1. Compile the Sketch

From this folder:

```powershell
.\compile.cmd
```

This compiles only. It does not upload.

### 2. Confirm Motor CAN Feedback

Power the ESC and CAN adapter correctly, then watch telemetry.

You should see the motor ID appear with a valid type, temperature, velocity,
and current.

If the motor does not appear:

- Check ESC ID.
- Check CAN wiring.
- Check CAN power.
- Check MCP2515 wiring.
- Check CAN termination.
- Check that the ESC is powered.

### 3. Stop Everything

Send:

```text
s
```

### 4. Set Motor Type

For Motor 1 as M3508/C620:

```text
k 1 1
```

For Motor 2 as M2006/C610:

```text
k 2 2
```

### 5. Attach the Encoder

For Motor 1 on mux channel 0:

```text
e 1 0
```

This saves the encoder mapping to EEPROM.

### 6. Verify Encoder Readings Before Moving

Manually rotate the joint slowly by hand if it is safe.

Watch `POS_D`.

It should change smoothly.

If it jumps:

- Check magnet alignment.
- Check AS5600 power.
- Check mux channel.
- Check I2C wiring.
- Check that only one mux channel is intended for that motor.

If it moves opposite the expected direction, remember that direction may need a
future firmware setting before closed-loop position control behaves correctly.

### 7. Zero the Encoder

Place the joint in the physical position you want to call zero.

Send:

```text
z 1
```

Telemetry should now show position near `0` degrees.

The motor should not move just because you zeroed.

### 8. Set Gentle PID Gains

Start small:

```text
pid 1 0.5 0.0 0.0
```

Then set a low position velocity limit:

```text
l 1 50
```

### 9. Try a Small Position Move

Use a tiny target first:

```text
p 1 5
```

Then:

```text
p 1 0
```

Only after small moves work should you try larger moves like:

```text
p 1 90
```

## PID Tuning Basics

This firmware uses cascaded control in position mode:

```text
position PID -> target output RPM -> velocity PID -> motor current
```

The position PID does not directly command current. It asks for an output
velocity. The velocity PID then asks for motor current.

### P Gain

P gain is the main response strength.

Too low:

- Motor moves slowly.
- Motor may not reach target.

Too high:

- Motor overshoots.
- Motor oscillates.
- Motor may move violently if feedback direction is wrong.

### I Gain

I gain corrects steady-state error over time.

For early testing, keep it at zero:

```text
pid 1 0.5 0.0 0.0
```

Add I only after direction, scaling, and P behavior are correct.

### D Gain

D gain damps fast changes.

Too little:

- Overshoot or oscillation.

Too much:

- Noise sensitivity.
- Jitter.

### A Conservative Tuning Path

1. Set `I = 0`.
2. Set `D = 0`.
3. Start with low `P`, such as `0.2`.
4. Command `p 1 5`.
5. Increase `P` slowly until it moves reliably.
6. Add a little `D` if it overshoots.
7. Add `I` only if needed.

Example sequence:

```text
s
l 1 50
pid 1 0.2 0.0 0.0
p 1 5
p 1 0
pid 1 0.5 0.0 0.0
p 1 5
p 1 0
pid 1 0.8 0.0 0.01
p 1 5
p 1 0
```

## Encoder Loss Behavior

If an encoder is enabled for a motor and I2C reads fail repeatedly, the
firmware marks that encoder invalid.

In position mode:

- The motor commands zero current.
- The position PID is cleared.
- The velocity PID is cleared.
- The controller does not silently fall back to the CAN encoder.

This is intentional. Falling back from absolute encoder position to motor CAN
position can cause a sudden coordinate-frame jump.

When encoder feedback becomes valid again:

- The latest AS5600 reading is accepted.
- The position target is rebased to the current measured position.
- The PID state is cleared.

This is meant to avoid a sudden jump when feedback returns.

## Why Zeroing Used To Move the Motor

Zeroing changes the coordinate frame.

For example, imagine this state before zeroing:

```text
actual encoder position = 40 degrees
target position = 90 degrees
```

The motor is trying to move from `40` to `90`.

If you now say "the current position is zero" but keep the old target:

```text
actual encoder position = 0 degrees
target position = 90 degrees
```

The motor now tries to move 90 degrees from the new zero. That looks like
zeroing caused motion.

The current code fixes this by rebasing the target to the current position when
`z` is used.

## Troubleshooting

### The Motor Moves When I Send `z`

Expected current behavior: it should not.

Check:

- Did the new firmware actually get uploaded?
- Did the Serial console print `ZERO SAVED`?
- Is the motor in direct current mode from an earlier `i` command?
- Is another controller sending commands?
- Is the mechanism backdriving or falling under gravity?

Send:

```text
s
```

Then zero again:

```text
z 1
```

### It Says 90 Degrees But the Joint Did Not Move 90 Degrees

Most likely causes:

- The AS5600 is not measuring the final joint shaft.
- There is gearing between the encoder and the joint.
- The encoder magnet is slipping.
- The motor output gear ratio is different than assumed.
- The joint moved, but the encoder shaft moved a different amount.

The firmware currently treats AS5600 degrees as joint degrees. If that is not
mechanically true, a future encoder scale setting is needed.

### The Motor Moves in the Wrong Direction

Most likely causes:

- Encoder direction is opposite motor direction.
- Motor direction is opposite expected direction.
- The mechanism reverses direction through gears or belts.

Stop immediately:

```text
s
```

Then test with very small current or velocity commands while watching whether
`POS_D` increases or decreases.

### Encoder Shows BAD or Error

Check:

- SDA is on pin `6`.
- SCL is on pin `7`.
- The code is using `Wire1`.
- I2C clock is `100 kHz`.
- PCA9548A address is `0x70`.
- AS5600 is powered.
- The selected mux channel is correct.
- Pullups are present on the I2C bus.
- Wires are short enough and not routed near noisy motor wiring.

Common I2C error meanings:

| Error | Meaning |
|---:|---|
| `2` | NACK on address |
| `3` | NACK on data |
| `4` | Other I2C error |
| `98` | Invalid mux channel in firmware |
| `99` | Did not receive both AS5600 bytes |

### Motor Does Not Spin in Velocity Mode

Check:

- ESC is powered.
- CAN wiring is correct.
- CAN adapter is powered.
- Motor ID matches the command.
- Telemetry shows the motor as active.
- You are not in stop mode after sending `s`.
- The current limit is not being hit immediately.

Try:

```text
v 1 50
```

Then stop:

```text
s
```

### Motor Works in Velocity But Not Position

Check:

- Encoder is valid if external feedback is enabled.
- `POS_D` changes smoothly when the joint is moved.
- Position PID gains are not zero.
- Position velocity limit is not too low.
- Encoder direction matches motor direction.

Try:

```text
l 1 50
pid 1 0.5 0.0 0.0
p 1 5
```

## Adding More Motors and Encoders

The firmware already creates 8 motor controller slots.

To add a second motor with an encoder:

1. Set the ESC ID to `2`.
2. Wire its AS5600 to another PCA9548A channel, for example channel `1`.
3. Configure motor type:

```text
k 2 2
```

4. Attach encoder:

```text
e 2 1
```

5. Move the joint to its physical zero.

```text
z 2
```

6. Tune PID:

```text
pid 2 0.5 0.0 0.0
```

7. Test small moves:

```text
p 2 5
p 2 0
```

## Known Limitations

The current firmware is a test platform, not the final robot arm controller.

Known limitations:

- Encoder direction is not yet configurable per motor.
- Encoder-to-joint gear ratio is not yet configurable per motor.
- Motor type is not yet persisted to EEPROM.
- Velocity PID gains are not yet tunable from Serial.
- Current limits are based on UdonLibrary ESC ranges, not arm-specific joint
  safety limits.
- There is no soft joint limit table yet.
- There is no homing sequence beyond AS5600 zeroing.
- There is no ROS 2 packet protocol yet.

For a robot arm, the next high-value additions are:

1. Persisted encoder direction per motor.
2. Persisted encoder scale or joint gear ratio per motor.
3. Persisted motor type per motor.
4. Per-joint min/max soft limits.
5. A structured binary or line-based protocol for ROS 2.

## Build Instructions

### Prerequisites

You need:

- Arduino IDE or Arduino CLI.
- The RP2040/RP2350 Arduino core installed.
- UdonLibrary available to the Arduino build.
- The XIAO RP2350 board package installed.

The compile script expects this fully qualified board name:

```text
rp2040:rp2040:seeed_xiao_rp2350
```

### Compile

From this folder:

```powershell
.\compile.cmd
```

or:

```powershell
.\compile.ps1
```

Successful compile output should show program storage and dynamic memory usage.

### Upload

This README does not include an upload command because upload method depends on
your local port and bootloader state. Be careful not to open a Serial monitor
from another tool while uploading or testing.

## Future ROS 2 / MoveIt Direction

The current Serial commands are intentionally simple for manual testing.

For ROS 2 integration, the firmware will eventually need:

- A deterministic command packet format.
- A deterministic telemetry packet format.
- Joint names or fixed joint indexes.
- Position, velocity, and effort state reporting.
- Command timeout handling.
- Per-joint limits.
- Persistent calibration data.
- A host-side ROS 2 hardware interface.

The current structure helps with that because each motor already has a
`MotorController` wrapper and the main sketch already separates:

- CAN updates.
- Encoder sampling.
- Control updates.
- Command parsing.
- Telemetry output.

That separation makes it easier to replace the current human Serial parser with
a ROS-facing protocol later.

## Quick Reference

Most common first-session commands:

```text
s
k 1 1
e 1 0
z 1
l 1 50
pid 1 0.5 0.0 0.0
p 1 5
p 1 0
s
```

Most common emergency command:

```text
s
```

Most important telemetry fields:

```text
POS_D    current position in degrees
TGT_D    target position in degrees
FB       feedback source
ENC      encoder status and mux channel
TMP      temperature in Celsius
CURR     torque current in mA
```
