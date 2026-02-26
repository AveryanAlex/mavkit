"""Tests for tlog parser (TlogFile, TlogEntry).

These tests require a real tlog file at ``mav.tlog`` (212 MB, gitignored).
All tests are skipped when the file is not present.
"""

import asyncio
import os

import pytest

import mavkit

TLOG_PATH = os.path.join(os.path.dirname(__file__), "..", "mav.tlog")
HAS_TLOG = os.path.isfile(TLOG_PATH)
skip_no_tlog = pytest.mark.skipif(not HAS_TLOG, reason="mav.tlog not present")


def run(async_fn, *args):
    """Run an async callable synchronously via asyncio.run()."""

    async def _wrapper():
        return await async_fn(*args)

    return asyncio.run(_wrapper())


class TestTlogOpen:
    @skip_no_tlog
    def test_open_existing_file(self):
        tlog = run(mavkit.TlogFile.open, TLOG_PATH)
        assert tlog is not None

    def test_open_nonexistent_raises(self):
        with pytest.raises(mavkit.MavkitError):
            run(mavkit.TlogFile.open, "/nonexistent/path.tlog")


class TestTlogEntries:
    @skip_no_tlog
    def test_entries_returns_list(self):
        async def _test():
            tlog = await mavkit.TlogFile.open(TLOG_PATH)
            return await tlog.entries()

        entries = asyncio.run(_test())
        assert isinstance(entries, list)
        assert len(entries) > 0

    @skip_no_tlog
    def test_entry_has_required_fields(self):
        async def _test():
            tlog = await mavkit.TlogFile.open(TLOG_PATH)
            return await tlog.entries()

        entries = asyncio.run(_test())
        entry = entries[0]
        assert isinstance(entry.timestamp_usec, int)
        assert isinstance(entry.message_id, int)
        assert isinstance(entry.message_name, str)
        assert len(entry.message_name) > 0

    @skip_no_tlog
    def test_entry_message_json_is_valid(self):
        import json

        async def _test():
            tlog = await mavkit.TlogFile.open(TLOG_PATH)
            return await tlog.entries()

        entries = asyncio.run(_test())
        entry = entries[0]
        data = json.loads(entry.message_json())
        assert isinstance(data, dict)

    @skip_no_tlog
    def test_entries_timestamps_are_monotonic(self):
        async def _test():
            tlog = await mavkit.TlogFile.open(TLOG_PATH)
            return await tlog.entries()

        entries = asyncio.run(_test())
        timestamps = [e.timestamp_usec for e in entries[:100]]
        assert timestamps == sorted(timestamps)


class TestTlogTimeRange:
    @skip_no_tlog
    def test_time_range_returns_tuple(self):
        async def _test():
            tlog = await mavkit.TlogFile.open(TLOG_PATH)
            return await tlog.time_range()

        result = asyncio.run(_test())
        assert result is not None
        first, last = result
        assert isinstance(first, int)
        assert isinstance(last, int)
        assert first <= last

    @skip_no_tlog
    def test_time_range_matches_entries(self):
        async def _test():
            tlog = await mavkit.TlogFile.open(TLOG_PATH)
            time_range = await tlog.time_range()
            entries = await tlog.entries()
            return time_range, entries

        time_range, entries = asyncio.run(_test())
        assert time_range is not None
        first, last = time_range
        assert entries[0].timestamp_usec == first
        assert entries[-1].timestamp_usec == last


class TestTlogSeek:
    @skip_no_tlog
    def test_seek_to_timestamp_returns_subset(self):
        async def _test():
            tlog = await mavkit.TlogFile.open(TLOG_PATH)
            time_range = await tlog.time_range()
            assert time_range is not None
            first, last = time_range
            midpoint = (first + last) // 2
            return await tlog.seek_to_timestamp(midpoint), midpoint

        entries, midpoint = asyncio.run(_test())
        assert isinstance(entries, list)
        assert len(entries) > 0
        assert entries[0].timestamp_usec >= midpoint

    @skip_no_tlog
    def test_seek_to_start_returns_all(self):
        async def _test():
            tlog = await mavkit.TlogFile.open(TLOG_PATH)
            all_entries = await tlog.entries()
            from_start = await tlog.seek_to_timestamp(0)
            return all_entries, from_start

        all_entries, from_start = asyncio.run(_test())
        assert len(from_start) == len(all_entries)

    @skip_no_tlog
    def test_seek_past_end_returns_empty(self):
        async def _test():
            tlog = await mavkit.TlogFile.open(TLOG_PATH)
            time_range = await tlog.time_range()
            assert time_range is not None
            _, last = time_range
            return await tlog.seek_to_timestamp(last + 1_000_000)

        entries = asyncio.run(_test())
        assert len(entries) == 0
