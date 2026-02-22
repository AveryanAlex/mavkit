#!/usr/bin/env python3

import socket
import sys
import time


def main() -> int:
    if len(sys.argv) not in (3, 4):
        print(
            "usage: wait_udp_telemetry.py <bind_host> <bind_port> [timeout_s]",
            file=sys.stderr,
        )
        return 2

    host = sys.argv[1]
    port = int(sys.argv[2])
    timeout_s = int(sys.argv[3]) if len(sys.argv) == 4 else 180

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((host, port))
    sock.settimeout(1)

    deadline = time.time() + timeout_s
    ok = False
    try:
        while time.time() < deadline:
            try:
                data, _addr = sock.recvfrom(4096)
                if data:
                    ok = True
                    break
            except TimeoutError:
                pass
    finally:
        sock.close()

    print("UDP telemetry ready" if ok else "Timed out waiting for UDP telemetry")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
