from __future__ import annotations

import argparse
import sys

from . import protocol
from .serial_client import SerialClient


def add_common(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--port", default="/dev/ttyACM0", help="RP2350 serial port inside Linux/Docker")
    parser.add_argument("--baud", type=int, default=921600, help="Serial baud rate")
    parser.add_argument("--timeout", type=float, default=1.0, help="Serial response timeout in seconds")


def motor_arg(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--motor", type=int, required=True, choices=range(1, 9), metavar="1-8")


def print_ack(ack: protocol.ConfigAck) -> int:
    command = protocol.CONFIG_TEXT.get(ack.command, f"command {ack.command}")
    status = protocol.STATUS_TEXT.get(ack.status, f"status {ack.status}")
    print(f"{command}: motor {ack.motor_id}: {status}")
    return 0 if ack.status == protocol.STATUS_OK else 2


def command_config_encoder(args: argparse.Namespace) -> int:
    with SerialClient(args.port, args.baud, args.timeout) as client:
        ack = client.send_config_and_wait_ack(
            command=protocol.CONFIG_ENABLE_ENCODER,
            motor_id=args.motor,
            encoder_channel=args.channel,
        )
    return print_ack(ack)


def command_disable_encoder(args: argparse.Namespace) -> int:
    with SerialClient(args.port, args.baud, args.timeout) as client:
        ack = client.send_config_and_wait_ack(
            command=protocol.CONFIG_DISABLE_ENCODER,
            motor_id=args.motor,
        )
    return print_ack(ack)


def command_zero_encoder(args: argparse.Namespace) -> int:
    with SerialClient(args.port, args.baud, args.timeout) as client:
        ack = client.send_config_and_wait_ack(
            command=protocol.CONFIG_ZERO_CURRENT_POSITION,
            motor_id=args.motor,
        )
    return print_ack(ack)


def command_set_pid(args: argparse.Namespace) -> int:
    with SerialClient(args.port, args.baud, args.timeout) as client:
        ack = client.send_config_and_wait_ack(
            command=protocol.CONFIG_SET_POSITION_PID,
            motor_id=args.motor,
            position_pid_p=args.p,
            position_pid_i=args.i,
            position_pid_d=args.d,
        )
    return print_ack(ack)


def command_set_limit(args: argparse.Namespace) -> int:
    with SerialClient(args.port, args.baud, args.timeout) as client:
        ack = client.send_config_and_wait_ack(
            command=protocol.CONFIG_SET_VELOCITY_LIMIT,
            motor_id=args.motor,
            velocity_limit_rpm=args.rpm,
        )
    return print_ack(ack)


def command_set_motor_type(args: argparse.Namespace) -> int:
    motor_type = 1 if args.type in ("m3508", "3508", "c620", "1") else 2
    with SerialClient(args.port, args.baud, args.timeout) as client:
        ack = client.send_config_and_wait_ack(
            command=protocol.CONFIG_SET_MOTOR_TYPE,
            motor_id=args.motor,
            motor_type=motor_type,
        )
    return print_ack(ack)


def command_telemetry_once(args: argparse.Namespace) -> int:
    with SerialClient(args.port, args.baud, args.timeout) as client:
        telemetry = client.request_telemetry()

    print(f"MCU time: {telemetry.mcu_time_ms} ms")
    print(f"Last command age: {telemetry.last_command_age_ms} ms")
    print(f"Watchdog tripped: {bool(telemetry.watchdog_tripped)}")
    print("id active mode enc_cfg enc_ok ch i2c_err pos_deg vel_rpm current_mA temp_C")
    for index, motor in enumerate(telemetry.motors, start=1):
        if index > telemetry.motor_count:
            break
        print(
            f"{index:2d} {motor.active:6d} {motor.mode:4d} {motor.encoder_configured:7d} "
            f"{motor.encoder_valid:6d} {motor.encoder_channel:2d} {motor.i2c_error:7d} "
            f"{motor.position_deg:8.3f} {motor.velocity_rpm:7.2f} "
            f"{motor.current_ma:10.1f} {motor.temperature_c:6.1f}"
        )
    return 0


def command_safe_halt(args: argparse.Namespace) -> int:
    with SerialClient(args.port, args.baud, args.timeout) as client:
        client.send_halt()
    print("halt packet sent")
    return 0


def build_parser(prog: str = "robomaster_arm_config") -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog=prog,
        description="Configure and inspect the RP2350 RoboMaster arm firmware over the binary COBS protocol.",
    )
    add_common(parser)
    subparsers = parser.add_subparsers(dest="command", required=True)

    p = subparsers.add_parser("config-encoder", help="Enable an AS5600 encoder for a motor")
    motor_arg(p)
    p.add_argument("--channel", type=int, required=True, choices=range(0, 8), metavar="0-7")
    p.set_defaults(func=command_config_encoder)

    p = subparsers.add_parser("disable-encoder", help="Disable external encoder feedback for a motor")
    motor_arg(p)
    p.set_defaults(func=command_disable_encoder)

    p = subparsers.add_parser("zero-encoder", help="Save current encoder angle as zero for a motor")
    motor_arg(p)
    p.set_defaults(func=command_zero_encoder)

    p = subparsers.add_parser("set-pid", help="Set position PID gains for one motor")
    motor_arg(p)
    p.add_argument("--p", type=float, required=True)
    p.add_argument("--i", type=float, required=True)
    p.add_argument("--d", type=float, required=True)
    p.set_defaults(func=command_set_pid)

    p = subparsers.add_parser("set-limit", help="Set position-loop velocity limit in output RPM")
    motor_arg(p)
    p.add_argument("--rpm", type=float, required=True)
    p.set_defaults(func=command_set_limit)

    p = subparsers.add_parser("set-motor-type", help="Set motor type/gearing")
    motor_arg(p)
    p.add_argument("--type", required=True, choices=["m3508", "3508", "c620", "1", "m2006", "2006", "c610", "2"])
    p.set_defaults(func=command_set_motor_type)

    p = subparsers.add_parser("telemetry-once", help="Request and print one telemetry packet")
    p.set_defaults(func=command_telemetry_once)

    p = subparsers.add_parser("safe-halt", help="Send an immediate halt packet")
    p.set_defaults(func=command_safe_halt)

    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    return run_command(args.func, args)


def run_command(func, args: argparse.Namespace) -> int:
    try:
        return func(args)
    except TimeoutError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 3
    except (OSError, RuntimeError) as exc:
        print(f"ERROR: serial port problem: {exc}", file=sys.stderr)
        return 4


def _run_single(subcommand: str, argv: list[str] | None = None) -> int:
    return main(["--port", "/dev/ttyACM0", subcommand] + (sys.argv[1:] if argv is None else argv))


def config_encoder_main() -> int:
    parser = argparse.ArgumentParser(description="Enable an AS5600 encoder for a motor")
    add_common(parser)
    motor_arg(parser)
    parser.add_argument("--channel", type=int, required=True, choices=range(0, 8), metavar="0-7")
    return run_command(command_config_encoder, parser.parse_args())


def disable_encoder_main() -> int:
    parser = argparse.ArgumentParser(description="Disable external encoder feedback for a motor")
    add_common(parser)
    motor_arg(parser)
    return run_command(command_disable_encoder, parser.parse_args())


def zero_encoder_main() -> int:
    parser = argparse.ArgumentParser(description="Save current encoder angle as zero for a motor")
    add_common(parser)
    motor_arg(parser)
    return run_command(command_zero_encoder, parser.parse_args())


def set_pid_main() -> int:
    parser = argparse.ArgumentParser(description="Set position PID gains for one motor")
    add_common(parser)
    motor_arg(parser)
    parser.add_argument("--p", type=float, required=True)
    parser.add_argument("--i", type=float, required=True)
    parser.add_argument("--d", type=float, required=True)
    return run_command(command_set_pid, parser.parse_args())


def set_limit_main() -> int:
    parser = argparse.ArgumentParser(description="Set position-loop velocity limit in output RPM")
    add_common(parser)
    motor_arg(parser)
    parser.add_argument("--rpm", type=float, required=True)
    return run_command(command_set_limit, parser.parse_args())


def set_motor_type_main() -> int:
    parser = argparse.ArgumentParser(description="Set motor type/gearing")
    add_common(parser)
    motor_arg(parser)
    parser.add_argument("--type", required=True, choices=["m3508", "3508", "c620", "1", "m2006", "2006", "c610", "2"])
    return run_command(command_set_motor_type, parser.parse_args())


def telemetry_once_main() -> int:
    parser = argparse.ArgumentParser(description="Request and print one telemetry packet")
    add_common(parser)
    return run_command(command_telemetry_once, parser.parse_args())


def safe_halt_main() -> int:
    parser = argparse.ArgumentParser(description="Send an immediate halt packet")
    add_common(parser)
    return run_command(command_safe_halt, parser.parse_args())


if __name__ == "__main__":
    raise SystemExit(main())
