#!/usr/bin/env python3

from __future__ import annotations

import argparse
import asyncio
import base64
import contextlib
import hashlib
import sys


GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"
MAX_FRAME_PAYLOAD = 16 * 1024 * 1024


class WebSocketClosed(Exception):
    pass


async def read_http_request(reader: asyncio.StreamReader) -> dict[str, str]:
    request = bytearray()
    while b"\r\n\r\n" not in request:
        chunk = await reader.read(1024)
        if not chunk:
            raise WebSocketClosed("client closed before websocket handshake")
        request.extend(chunk)
        if len(request) > 16 * 1024:
            raise ValueError("websocket handshake exceeded 16 KiB")

    lines = request.decode("latin1").split("\r\n")
    headers: dict[str, str] = {}
    for line in lines[1:]:
        if not line:
            break
        name, separator, value = line.partition(":")
        if separator:
            headers[name.strip().lower()] = value.strip()
    return headers


async def websocket_handshake(
    reader: asyncio.StreamReader, writer: asyncio.StreamWriter
) -> None:
    headers = await read_http_request(reader)
    key = headers.get("sec-websocket-key")
    if key is None:
        raise ValueError("missing Sec-WebSocket-Key header")

    accept = base64.b64encode(hashlib.sha1(f"{key}{GUID}".encode()).digest()).decode()
    response = (
        "HTTP/1.1 101 Switching Protocols\r\n"
        "Upgrade: websocket\r\n"
        "Connection: Upgrade\r\n"
        f"Sec-WebSocket-Accept: {accept}\r\n"
        "\r\n"
    )
    writer.write(response.encode("ascii"))
    await writer.drain()


async def read_frame(reader: asyncio.StreamReader) -> tuple[bool, int, bytes]:
    try:
        first, second = await reader.readexactly(2)
    except asyncio.IncompleteReadError as exc:
        raise WebSocketClosed("client closed websocket") from exc

    fin = bool(first & 0x80)
    opcode = first & 0x0F
    masked = bool(second & 0x80)
    length = second & 0x7F

    if length == 126:
        length = int.from_bytes(await reader.readexactly(2), "big")
    elif length == 127:
        length = int.from_bytes(await reader.readexactly(8), "big")

    if length > MAX_FRAME_PAYLOAD:
        raise ValueError(f"websocket frame too large: {length} bytes")

    mask = await reader.readexactly(4) if masked else b""
    payload = bytearray(await reader.readexactly(length))
    if masked:
        for index, byte in enumerate(payload):
            payload[index] = byte ^ mask[index % 4]

    return fin, opcode, bytes(payload)


async def send_frame(writer: asyncio.StreamWriter, opcode: int, payload: bytes = b"") -> None:
    header = bytearray([0x80 | opcode])
    length = len(payload)
    if length < 126:
        header.append(length)
    elif length <= 0xFFFF:
        header.append(126)
        header.extend(length.to_bytes(2, "big"))
    else:
        header.append(127)
        header.extend(length.to_bytes(8, "big"))

    writer.write(header)
    writer.write(payload)
    await writer.drain()


async def read_binary_message(
    reader: asyncio.StreamReader, writer: asyncio.StreamWriter
) -> bytes:
    fragments = bytearray()
    fragmented_binary = False

    while True:
        fin, opcode, payload = await read_frame(reader)

        if opcode == 0x8:
            await send_frame(writer, 0x8, payload[:125])
            raise WebSocketClosed("client sent websocket close")
        if opcode == 0x9:
            await send_frame(writer, 0xA, payload[:125])
            continue
        if opcode == 0xA:
            continue
        if opcode == 0x2:
            if fin:
                return payload
            fragments.extend(payload)
            fragmented_binary = True
            continue
        if opcode == 0x0 and fragmented_binary:
            fragments.extend(payload)
            if fin:
                return bytes(fragments)
            continue

        raise ValueError(f"unsupported websocket opcode: {opcode}")


async def websocket_to_tcp(
    ws_reader: asyncio.StreamReader,
    ws_writer: asyncio.StreamWriter,
    tcp_writer: asyncio.StreamWriter,
) -> None:
    while True:
        payload = await read_binary_message(ws_reader, ws_writer)
        if payload:
            tcp_writer.write(payload)
            await tcp_writer.drain()


async def tcp_to_websocket(
    tcp_reader: asyncio.StreamReader, ws_writer: asyncio.StreamWriter
) -> None:
    while True:
        chunk = await tcp_reader.read(4096)
        if not chunk:
            await send_frame(ws_writer, 0x8)
            return
        await send_frame(ws_writer, 0x2, chunk)


async def handle_client(
    ws_reader: asyncio.StreamReader,
    ws_writer: asyncio.StreamWriter,
    target_host: str,
    target_port: int,
) -> None:
    tcp_writer: asyncio.StreamWriter | None = None
    try:
        await websocket_handshake(ws_reader, ws_writer)
        tcp_reader, tcp_writer = await asyncio.open_connection(target_host, target_port)

        tasks = {
            asyncio.create_task(websocket_to_tcp(ws_reader, ws_writer, tcp_writer)),
            asyncio.create_task(tcp_to_websocket(tcp_reader, ws_writer)),
        }
        done, pending = await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
        for task in pending:
            task.cancel()
        await asyncio.gather(*pending, return_exceptions=True)
        for task in done:
            task.result()
    except (asyncio.CancelledError, WebSocketClosed):
        pass
    except Exception as exc:  # noqa: BLE001 - proxy logs all connection failures for diagnostics.
        print(f"websocket proxy connection error: {exc}", file=sys.stderr, flush=True)
    finally:
        if tcp_writer is not None:
            tcp_writer.close()
            with contextlib.suppress(Exception):
                await tcp_writer.wait_closed()
        ws_writer.close()
        with contextlib.suppress(Exception):
            await ws_writer.wait_closed()


async def main_async(args: argparse.Namespace) -> None:
    server = await asyncio.start_server(
        lambda reader, writer: handle_client(reader, writer, args.target_host, args.target_port),
        args.listen_host,
        args.listen_port,
    )
    sockets = server.sockets or []
    bound = sockets[0].getsockname() if sockets else (args.listen_host, args.listen_port)
    print(
        f"WebSocket proxy ready at ws://{bound[0]}:{bound[1]} -> tcp:{args.target_host}:{args.target_port}",
        flush=True,
    )

    async with server:
        await server.serve_forever()


def main() -> int:
    parser = argparse.ArgumentParser(description="Bridge browser WebSocket bytes to a TCP endpoint.")
    parser.add_argument("--listen-host", default="127.0.0.1")
    parser.add_argument("--listen-port", type=int, required=True)
    parser.add_argument("--target-host", required=True)
    parser.add_argument("--target-port", type=int, required=True)
    args = parser.parse_args()

    try:
        asyncio.run(main_async(args))
    except KeyboardInterrupt:
        return 130
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
