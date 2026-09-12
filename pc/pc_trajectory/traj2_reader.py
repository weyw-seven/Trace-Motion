"""
Strict canonical TRJ2 reader / verifier.

Unlike a future firmware compatibility reader, this PC reader intentionally
accepts only the canonical B6 representation:

    header_size == 32
    record_size == 44
    exact file length
    flags/reserved == 0
    unused payload slots == 0
    known record types only
"""

from __future__ import annotations

from pathlib import Path

from .traj2_format import (
    TRJ2_HEADER_SIZE,
    TRJ2_RECORD_SIZE,
    Trj2File,
    header_from_raw,
    record_from_raw,
    unpack_header_raw,
    unpack_record_raw,
)
from .traj2_validate import (
    Trj2ValidationError,
    validate_raw_header,
    validate_raw_record,
    validate_trj2_file,
)


class Trj2DecodeError(Trj2ValidationError):
    """Raised for malformed or non-canonical TRJ2 bytes."""


def decode_trj2(data: bytes | bytearray | memoryview) -> Trj2File:
    try:
        blob = bytes(data)
    except Exception as exc:
        raise Trj2DecodeError(
            "TRJ2 data must be bytes-like"
        ) from exc

    if len(blob) < TRJ2_HEADER_SIZE:
        raise Trj2DecodeError(
            f"TRJ2 file shorter than {TRJ2_HEADER_SIZE}-byte header"
        )

    try:
        raw_header = unpack_header_raw(
            blob[:TRJ2_HEADER_SIZE]
        )
        validate_raw_header(raw_header)
    except (ValueError, Trj2ValidationError) as exc:
        raise Trj2DecodeError(str(exc)) from exc

    expected_size = (
        TRJ2_HEADER_SIZE
        + raw_header.record_count * TRJ2_RECORD_SIZE
    )

    if len(blob) != expected_size:
        raise Trj2DecodeError(
            f"canonical TRJ2 file size mismatch: "
            f"actual={len(blob)}, expected={expected_size}"
        )

    records = []

    for record_index in range(raw_header.record_count):
        offset = (
            TRJ2_HEADER_SIZE
            + record_index * TRJ2_RECORD_SIZE
        )
        record_bytes = blob[
            offset : offset + TRJ2_RECORD_SIZE
        ]

        try:
            raw_record = unpack_record_raw(record_bytes)
            validate_raw_record(
                raw_record,
                record_index=record_index,
            )
            records.append(record_from_raw(raw_record))
        except (ValueError, Trj2ValidationError) as exc:
            raise Trj2DecodeError(
                f"TRJ2 record {record_index}: {exc}"
            ) from exc

    file = Trj2File(
        header=header_from_raw(raw_header),
        records=tuple(records),
    )

    try:
        validate_trj2_file(file)
    except Trj2ValidationError as exc:
        raise Trj2DecodeError(str(exc)) from exc

    return file


def read_trj2(
    path: str | Path,
) -> Trj2File:
    return decode_trj2(Path(path).read_bytes())


def format_trj2(file: Trj2File) -> str:
    validate_trj2_file(file)

    lines = [
        "TRJ2 V2",
        (
            "start="
            f"({file.header.start_x_mm:.3f}, "
            f"{file.header.start_y_mm:.3f}, "
            f"{file.header.start_yaw_deg:.3f} deg)"
        ),
        f"records={file.record_count}",
        "",
    ]

    for index, record in enumerate(file.records):
        name = type(record).__name__.removeprefix("Trj2").removesuffix(
            "Record"
        )

        lines.append(f"[{index}] {name}")

        if hasattr(record, "speed_mm_s"):
            lines.append(
                f"    speed={record.speed_mm_s:.3f}"
            )
            lines.append(
                "    acceleration="
                f"{record.acceleration_mm_s2:.3f}"
            )

        if name == "Line":
            lines.append(
                "    end="
                f"({record.end_x_mm:.3f}, "
                f"{record.end_y_mm:.3f})"
            )

        elif name == "Circle":
            lines.append(
                "    center="
                f"({record.center_x_mm:.3f}, "
                f"{record.center_y_mm:.3f})"
            )
            lines.append(
                f"    radius={record.radius_mm:.3f}"
            )
            lines.append(
                f"    start_angle={record.start_angle_deg:.3f}"
            )
            lines.append(
                f"    sweep={record.sweep_deg:.3f}"
            )

        elif name == "CubicBezier":
            lines.append(
                "    control1="
                f"({record.control1_x_mm:.3f}, "
                f"{record.control1_y_mm:.3f})"
            )
            lines.append(
                "    control2="
                f"({record.control2_x_mm:.3f}, "
                f"{record.control2_y_mm:.3f})"
            )
            lines.append(
                "    end="
                f"({record.end_x_mm:.3f}, "
                f"{record.end_y_mm:.3f})"
            )

        elif name == "Wait":
            lines.append(
                f"    duration={record.duration_s:.3f} s"
            )

        lines.append("")

    return "\n".join(lines).rstrip()


__all__ = [
    "Trj2DecodeError",
    "decode_trj2",
    "format_trj2",
    "read_trj2",
]
