import json
import time
import serial

PORT = "/dev/ttyTHS1"
BAUD = 115200
# source venv/bin/activate


def send(ser, cmd):
    msg = json.dumps(cmd)
    ser.write((msg + "\n").encode("utf-8"))
    ser.flush()
    print(">>", msg)


def main():
    with serial.Serial(PORT, BAUD, timeout=1) as ser:
        time.sleep(1.5)  # let serial settle

        # Center
        send(ser, {"T": 133, "X": 0, "Y": 0, "SPD": 0, "ACC": 0})
        time.sleep(1.5)

        # Pan right
        send(ser, {"T": 133, "X": 30, "Y": 0, "SPD": 50, "ACC": 50})
        time.sleep(1.5)

        # Pan left
        send(ser, {"T": 133, "X": -30, "Y": 0, "SPD": 50, "ACC": 50})
        time.sleep(1.5)

        # Tilt up
        send(ser, {"T": 133, "X": 0, "Y": 20, "SPD": 50, "ACC": 50})
        time.sleep(1.5)

        # Tilt down
        send(ser, {"T": 133, "X": 0, "Y": -20, "SPD": 50, "ACC": 50})
        time.sleep(1.5)

        # Back to center
        send(ser, {"T": 133, "X": 0, "Y": 0, "SPD": 50, "ACC": 50})
        time.sleep(1.0)


if __name__ == "__main__":
    main()
