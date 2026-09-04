import sys, time, serial
port = sys.argv[1]
ser = serial.Serial(port, 115200, timeout=0.1)

t0 = time.time()
buf = bytearray()
while time.time() - t0 < 2.0:
    buf += ser.read(4096)

ser.close()
print("bytes:", len(buf))
print("first 200 hex:", buf[:200].hex(" "))
print("first 200 ascii:", buf[:200].decode("utf-8", errors="replace"))
