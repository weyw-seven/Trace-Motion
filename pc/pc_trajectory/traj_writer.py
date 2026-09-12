from __future__ import annotations

from pathlib import Path

from .traj_format import HEADER_SIZE, RECORD_SIZE, Trajectory, pack_header, pack_segment
from .traj_validate import validate_trajectory


def encode_traj(trajectory: Trajectory) -> bytes:
    """
    Validate and serialize one canonical TRJ1 V1 file.
    """
    validate_trajectory(trajectory)

    segment_count = len(trajectory.segments)
    chunks = [pack_header(trajectory.header, segment_count)]
    chunks.extend(pack_segment(segment) for segment in trajectory.segments)

    data = b"".join(chunks)
    expected_size = HEADER_SIZE + segment_count * RECORD_SIZE

    if len(data) != expected_size:
        raise AssertionError(
            f"internal size error: got {len(data)}, expected {expected_size}"
        )

    return data


def write_traj(trajectory: Trajectory, path: str | Path) -> None:
    """
    Validate and atomically-ish write one canonical TRJ1 V1 binary file.

    The complete byte stream is produced before the target is opened, so a
    validation/encoding failure cannot leave a partially-written new file.
    """
    data = encode_traj(trajectory)
    Path(path).write_bytes(data)
