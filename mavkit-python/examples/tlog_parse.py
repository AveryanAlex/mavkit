"""Parse a TLOG file and print all entries."""

import asyncio
import json
import sys

import mavkit


async def main():
    if len(sys.argv) < 2:
        print("usage: tlog_parse.py <path.tlog>")
        sys.exit(1)

    path = sys.argv[1]
    tlog = await mavkit.TlogFile.open(path)

    # Print time range
    time_range = await tlog.time_range()
    if time_range is not None:
        first, last = time_range
        print(f"time range: {first} .. {last} usec")
    else:
        print("empty file")
        return

    # Print all entries
    entries = await tlog.entries()
    for entry in entries:
        fields = json.loads(entry.message_json())
        print(f"[{entry.timestamp_usec}] {entry.message_name} (id={entry.message_id})")
        print(f"  {fields}")

    print(f"{len(entries)} entries total")


asyncio.run(main())
