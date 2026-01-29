import serial, time, json

PORT = "/dev/ttyTHS1"
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=0.3)

def send(cmd):
    # Many Waveshare ESP32 firmwares expect CRLF
    line = json.dumps(cmd) + "\r\n"
    ser.write(line.encode("utf-8"))
    ser.flush()
    time.sleep(0.08)
    resp = ser.read(300)
    if resp:
        try:
            print("RX:", resp.decode("utf-8", errors="replace"))
        except Exception:
            print("RX (bytes):", resp)

# --- ID setting (ONLY PAN SERVO CONNECTED for this!) ---
send({"T":501, "raw":1, "new":2})

# --- torque off so you can hand-center ---
send({"T":210, "cmd":0})

# After you manually align camera straight forward/level:
send({"T":502, "id":1})  # tilt
send({"T":502, "id":2})  # pan

# Go to center to verify
send({"T":133, "X":0, "Y":0, "SPD":0, "ACC":0})

ser.close()

