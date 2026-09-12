"""
Canonical TRJ2 writer.

This writer emits exactly:
    32-byte header
    record_count x 44-byte records

No extension header, oversized records, or trailing bytes are emitted.
"""

from __future__ import annotations

from pathlib import Path

from .traj2_format import (
    TRJ2_HEADER_SIZE,
    TRJ2_RECORD_SIZE,
    Trj2File,
    pack_header,
    pack_record,
)
from .traj2_validate import validate_trj2_file


def encode_trj2(file: Trj2File) -> bytes:
    validate_trj2_file(file)

    header = pack_header(
        file.header,
        record_count=file.record_count,
    )

    records = b"".join(
        pack_record(record)
        for record in file.records
    )

    data = header + records

    expected_size = (
        TRJ2_HEADER_SIZE
        + file.record_count * TRJ2_RECORD_SIZE
    )

    if len(data) != expected_size:
        raise AssertionError(
            f"TRJ2 writer size invariant failed: "
            f"{len(data)} != {expected_size}"
        )

    return data


def write_trj2(
    file: Trj2File,
    path: str | Path,
) -> Path:
    output = Path(path)
    output.write_bytes(encode_trj2(file))
    return output


__all__ = [
    "encode_trj2",
    "write_trj2",
]
