#!/usr/bin/env python3
# Log per-channel load cell data to a CSV file.
#
#   python3 log_load_cell.py [output.csv]
import json
import socket
import sys
import time

SOCKET_PATH = "/run/klippy.sock"
LOAD_CELL_NAME = "load_cell_probe"
DEFAULT_PATH = "/user-resource/gcodes/load_cell_%s.csv" % (
    time.strftime("%Y%m%d%H%M%S"),
)
out_path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PATH

sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
sock.connect(SOCKET_PATH)

def send(msg):
    sock.send(json.dumps(msg, separators=(",", ":")).encode() + b"\x03")

send(
    {
        "id": 1,
        "method": "load_cell/dump_force",
        "params": {
            "load_cell": LOAD_CELL_NAME,
            "response_template": {},
        },
    }
)

buf = b""
rows = 0
out = open(out_path, "w", buffering=1)
print("logging to %s, press Ctrl-C to stop" % (out_path,))
try:
    while True:
        data = sock.recv(65536)
        if not data:
            break
        parts = (buf + data).split(b"\x03")
        buf = parts.pop()
        for part in parts:
            if not part:
                continue
            msg = json.loads(part)
            result = msg.get("result")
            if result is not None:
                header = result.get("header")
                if header:
                    out.write(",".join(header) + "\n")
                continue
            if "error" in msg:
                sys.exit("subscribe failed: %s" % (msg["error"],))
            for row in msg.get("params", {}).get("data", []):
                out.write(",".join(str(v) for v in row) + "\n")
                rows += 1
except KeyboardInterrupt:
    pass
finally:
    out.close()
    print("\nwrote %d samples to %s" % (rows, out_path))
