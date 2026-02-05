import serial, time, json

PORT = "/dev/ttyUSB0"    # <-- if using header UART
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=0.3, xonxoff=False, rtscts=False, dsrdtr=False)

def send(cmd, crlf=False):
    line = json.dumps(cmd) + ("\r\n" if crlf else "\n")
    ser.write(line.encode("utf-8"))
    ser.flush()
    time.sleep(0.08)
    rx = ser.read(400)
    if rx:
        print("RX:", rx.decode("utf-8", errors="replace"))

# --- one-time calibration sequence (do ID change with only PAN servo connected) ---
send({"T":501, "raw":1, "new":2})     # pan servo becomes ID 2

# connect both servos, then:
send({"T":210, "cmd":0})              # torque off so you can hand-center
# hand-align camera straight forward/level, then:
send({"T":502, "id":1})               # save middle for tilt
send({"T":502, "id":2})               # save middle for pan

# test movement:
send({"T":133, "X":0, "Y":0, "SPD":0, "ACC":0})
time.sleep(1)
send({"T":133, "X":30, "Y":0, "SPD":0, "ACC":0})
time.sleep(1)
send({"T":133, "X":-30, "Y":0, "SPD":0, "ACC":0})

ser.close()

