#!/usr/bin/env python3

import socket
import base64
import time

HOST = "110.78.0.54"
PORT = 2116
USER = "1118600009224"
PASSWORD = "CK79"
MOUNTPOINT = "VRS_RTCM32"

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.settimeout(10)

try:
    print(f"Connecting to {HOST}:{PORT}...")
    sock.connect((HOST, PORT))

    auth = base64.b64encode(f"{USER}:{PASSWORD}".encode()).decode()

    request = (
        f"GET /{MOUNTPOINT} HTTP/1.0\r\n"
        f"User-Agent: NTRIP PythonClient\r\n"
        f"Authorization: Basic {auth}\r\n"
        "\r\n"
    )

    sock.sendall(request.encode())

    header = sock.recv(1024)
    print("========== HEADER ==========")
    print(header.decode(errors="ignore"))

    if b"200 OK" not in header and b"ICY 200 OK" not in header:
        print("❌ Connection failed")
        exit()

    print("✅ Connected")
    print("Checking RTCM stream for 15 seconds...")

    total = 0
    start = time.time()

    while time.time() - start < 15:
        data = sock.recv(4096)

        if not data:
            print("❌ Server closed connection")
            break

        total += len(data)
        print(f"Received {len(data)} bytes")

    print("--------------------------------")
    print(f"Total RTCM bytes = {total}")

    if total == 0:
        print("❌ No RTCM received")
    else:
        print("✅ RTCM stream is alive")

except Exception as e:
    print(e)

finally:
    sock.close()