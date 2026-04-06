from serial.tools import list_ports
from serial import Serial

print("Available ports:")
ports = list(list_ports.comports())
for p in ports:
    print(f" - {p.device} | {p.description}")

target = "COM5"  # change if needed
candidates = [target, r"\\.\\" + target]  # e.g. "COM6", "\\.\COM6"

for name in candidates:
    try:
        print(f"\nTrying to open: {name}")
        s = Serial(name, 38400, timeout=2)
        print("Opened OK:", s)
        s.close()
        print("Closed OK.")
    except Exception as e:
        print("Open failed:", repr(e))