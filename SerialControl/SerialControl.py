import serial


SERIAL_PORT = "/dev/ttyACM1" # Change this as needed. Usually it's /dev/ttyACM1 or /dev/ttyACM0
BAUD_RATE = 115200


def print_help():

    print()
    print("# Commands:")
    print("# v id rpm          -> velocity target in output rpm")
    print("# p id deg          -> position target in degrees")
    print("# i id mA           -> direct current command for low-level tests")
    print("# l id rpm          -> position loop velocity limit")
    print("# k id 1|2          -> motor gearing/type: 1=M3508/C620, 2=M2006/C610")
    print("# e id chan         -> attach AS5600 on PCA9548A mux channel")
    print("# ed id             -> disable external encoder feedback")
    print("# z id              -> save current AS5600 angle as zero degrees")
    print("# pid id p i d      -> tune position PID gains live, in degree units")
    print("# t                 -> toggle telemetry")
    print("# m                 -> toggle human/csv telemetry")
    print("# s                 -> stop all motors")
    print("# help              -> print this menu")
    print("# quit              -> exit")
    print()


ser = serial.Serial(
    SERIAL_PORT,
    BAUD_RATE,
    timeout=0.01
)

print(f"Connected to {SERIAL_PORT}")

print_help()

try:

    while True:

        cmd = input(">> ")

        cmd_clean = cmd.strip().lower()

        if cmd_clean == "quit":
            break

        if cmd_clean == "help":
            print_help()
            continue

        ser.write((cmd + "\n").encode("utf-8"))

except KeyboardInterrupt:

    pass

finally:

    ser.close()

    print("Disconnected")