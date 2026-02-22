import socket
import sys
import time


def main() -> int:
    if len(sys.argv) not in (3, 4):
        print("usage: wait_tcp.py <host> <port> [timeout_s]", file=sys.stderr)
        return 2

    host = sys.argv[1]
    port = int(sys.argv[2])
    timeout_s = int(sys.argv[3]) if len(sys.argv) == 4 else 120

    deadline = time.time() + timeout_s
    ok = False
    while time.time() < deadline:
        try:
            sock = socket.create_connection((host, port), timeout=1)
            sock.close()
            ok = True
            break
        except OSError:
            time.sleep(1)

    print("TCP ready" if ok else "Timed out waiting for TCP")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
