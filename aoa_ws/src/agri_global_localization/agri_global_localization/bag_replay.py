"""Replay legacy AOA solved poses through the current robust filter."""

import argparse
import math
from pathlib import Path
import subprocess
import tempfile
from typing import Iterable, Optional

from geometry_msgs.msg import PoseStamped
from rclpy.serialization import deserialize_message
import rosbag2_py

from .filter_core import AoaComplementaryFilter


TOPIC = '/ground_station/absolute_pose_local'


def percentile(values, fraction: float) -> float:
    ordered = sorted(float(value) for value in values)
    if not ordered:
        return 0.0
    position = fraction * (len(ordered) - 1)
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    alpha = position - lower
    return ordered[lower] * (1.0 - alpha) + ordered[upper] * alpha


def maximum_step(points: Iterable[tuple[float, float, float]]) -> float:
    points = list(points)
    return max((
        math.hypot(current[0] - previous[0], current[1] - previous[1])
        for previous, current in zip(points, points[1:])
    ), default=0.0)


def _read_database(uri: Path):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(uri), storage_id='sqlite3'),
        rosbag2_py.ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr'),
    )
    rows = []
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        if topic != TOPIC:
            continue
        message = deserialize_message(data, PoseStamped)
        position = message.pose.position
        rows.append((timestamp * 1e-9, position.x, position.y, position.z))
    return rows


def read_pose_rows(bag_path: str):
    path = Path(bag_path)
    metadata = path / 'metadata.yaml' if path.is_dir() else None
    compressed = (
        metadata is not None
        and metadata.exists()
        and 'compression_format: zstd' in metadata.read_text(encoding='utf-8'))
    if compressed:
        compressed_files = sorted(path.glob('*.db3.zstd'))
        if len(compressed_files) != 1:
            raise RuntimeError(f'{path} does not contain exactly one db3.zstd file')
        with tempfile.TemporaryDirectory(prefix='aoa_bag_replay_') as directory:
            database = Path(directory) / 'replay.db3'
            with database.open('wb') as output:
                subprocess.run(
                    ['zstd', '-dc', str(compressed_files[0])],
                    stdout=output, check=True)
            return _read_database(database)

    uri = path
    if path.is_dir():
        database_files = sorted(path.glob('*.db3'))
        if len(database_files) != 1:
            raise RuntimeError(f'{path} does not contain exactly one db3 file')
        uri = database_files[0]
    return _read_database(uri)


def replay(bag_path: str, truth: Optional[tuple[float, float]]) -> None:
    rows = read_pose_rows(bag_path)
    filter_ = AoaComplementaryFilter()
    raw_points = [(x, y, z) for _, x, y, z in rows]
    output_points = []
    output_stamps = []
    reasons = {}
    for stamp, x, y, z in rows:
        decision = filter_.update_measurement(
            (x, y, z), 0.0, stamp, xy_variance=0.04)
        reasons[decision.reason] = reasons.get(decision.reason, 0) + 1
        if decision.accepted:
            output_points.append(filter_.position)
            output_stamps.append(stamp)

    rejected = reasons.get('position_jump', 0)
    gaps = [
        current - previous
        for previous, current in zip(output_stamps, output_stamps[1:])]
    print(f'\n{bag_path}')
    print(f'samples={len(rows)} initialized={filter_.initialized}')
    print(f'raw_max_step_m={maximum_step(raw_points):.3f}')
    print(f'filtered_max_accepted_step_m={maximum_step(output_points):.3f}')
    print(f'accepted_outputs={len(output_points)} reasons={reasons}')
    print(f'rejection_fraction={rejected / max(len(rows), 1):.3f}')
    if gaps:
        print(
            f'accepted_gap_p95_s={percentile(gaps, 0.95):.3f} '
            f'accepted_gap_max_s={max(gaps):.3f}')
    if truth is not None and output_points:
        def rmse(points):
            errors = [
                (point[0] - truth[0]) ** 2 + (point[1] - truth[1]) ** 2
                for point in points]
            return math.sqrt(sum(errors) / len(errors))

        mean_x = sum(point[0] for point in output_points) / len(output_points)
        mean_y = sum(point[1] for point in output_points) / len(output_points)
        print(f'raw_rmse_m={rmse(raw_points):.3f}')
        print(f'filtered_rmse_m={rmse(output_points):.3f}')
        print(f'filtered_mean_xy=({mean_x:.3f},{mean_y:.3f})')


def main(args=None) -> None:
    parser = argparse.ArgumentParser(
        description='Replay legacy AOA local poses through the current filter')
    parser.add_argument('bags', nargs='+', help='rosbag2 directory paths')
    parser.add_argument('--truth-x', type=float)
    parser.add_argument('--truth-y', type=float)
    options = parser.parse_args(args)
    truth = None
    if options.truth_x is not None or options.truth_y is not None:
        if options.truth_x is None or options.truth_y is None:
            parser.error('--truth-x and --truth-y must be used together')
        truth = (options.truth_x, options.truth_y)
    for bag in options.bags:
        replay(bag, truth)


if __name__ == '__main__':
    main()
