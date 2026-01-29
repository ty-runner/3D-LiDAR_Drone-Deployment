import serial
import time

ser = serial.Serial("/dev/ttyTHS1", 115200, timeout=0.1)

def move_servo(servo_id, position, time_ms=500):
    position = max(0, min(1000, position))
    time_ms = max(0, min(30000, time_ms))

    params = [
        servo_id,
        7,              # length
        0x01,           # move command
        position & 0xFF,
        (position >> 8) & 0xFF,
        time_ms & 0xFF,
        (time_ms >> 8) & 0xFF
    ]

    checksum = (~sum(params)) & 0xFF
    packet = bytes([0x55, 0x55] + params + [checksum])

    ser.write(packet)

# Center position
move_servo(1, 500)
time.sleep(1)

# Move left
move_servo(1, 200)
time.sleep(1)

# Move right
move_servo(1, 800)

ser.close()

