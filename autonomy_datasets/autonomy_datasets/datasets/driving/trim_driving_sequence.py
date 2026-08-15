#!/usr/bin/env python3

# Copyright Thinking Cars GmbH
# SPDX-License-Identifier: Apache-2.0

import argparse
import csv
import json
from pathlib import Path


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Keep the first N sequential DrivIng samples referenced by timesync_info.csv.")
    parser.add_argument("dataset_dir", type=Path, help="Path to a DrivIng sequence directory")
    parser.add_argument("--count", type=int, default=100, help="Number of sequential samples to keep")
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Show what would be trimmed without modifying files",
    )
    return parser.parse_args()


def _unlink(path: Path, dry_run: bool) -> None:
    if dry_run:
        return
    path.unlink()


def _write_csv(path: Path, rows: list[list[str]], dry_run: bool) -> None:
    if dry_run:
        return
    with path.open("w", newline="") as file:
        writer = csv.writer(file)
        writer.writerows(rows)


def _write_json(path: Path, payload: dict, dry_run: bool) -> None:
    if dry_run:
        return
    with path.open("w") as file:
        json.dump(payload, file, indent=2)
        file.write("\n")


def _trim_annotations(path: Path, kept_timestamps: list[str], dry_run: bool) -> int:
    if not path.is_file():
        return 0

    with path.open() as file:
        annotations = json.load(file)

    kept_timestamp_set = set(kept_timestamps)
    filtered_tracks = []
    removed_tracks = 0

    for track in annotations.get("tracks", []):
        timestamps = track.get("timestamps")
        if not isinstance(timestamps, list):
            filtered_tracks.append(track)
            continue

        keep_indices = [index for index, timestamp in enumerate(timestamps) if str(timestamp) in kept_timestamp_set]
        if not keep_indices:
            removed_tracks += 1
            continue

        filtered_track = {}
        expected_len = len(timestamps)
        for key, value in track.items():
            if isinstance(value, list) and len(value) == expected_len:
                filtered_track[key] = [value[index] for index in keep_indices]
            else:
                filtered_track[key] = value
        filtered_tracks.append(filtered_track)

    annotations["timestamp"] = int(kept_timestamps[0])
    annotations["tracks"] = filtered_tracks
    _write_json(path, annotations, dry_run)
    return removed_tracks


def main() -> int:
    """Trim a DrivIng sequence directory down to the first N synchronized samples."""
    args = _parse_args()
    dataset_dir = args.dataset_dir
    csv_path = dataset_dir / "timesync_info.csv"

    with csv_path.open(newline="") as file:
        rows = list(csv.reader(file))

    if not rows or len(rows[0]) <= 1:
        raise SystemExit("timesync_info.csv is empty or malformed")

    sample_count = len(rows[0]) - 1
    if args.count <= 0:
        raise SystemExit("--count must be positive")
    if sample_count < args.count:
        raise SystemExit(f"Only {sample_count} samples are available; cannot keep {args.count}")

    kept_rows = [row[: args.count + 1] for row in rows]
    kept_timestamps = kept_rows[1][1:]
    first_timestamp = min(int(timestamp) for timestamp in kept_timestamps)
    last_timestamp = max(int(timestamp) for timestamp in kept_timestamps)

    sensor_keep_files: dict[str, set[str]] = {}
    deleted_indexed_files = 0
    for row in kept_rows[2:]:
        sensor = row[0]
        keep_files = {name for name in row[1:] if name}
        sensor_keep_files[sensor] = keep_files
        sensor_dir = dataset_dir / sensor
        if not sensor_dir.is_dir():
            continue
        for path in sensor_dir.iterdir():
            if path.is_file() and path.name not in keep_files:
                _unlink(path, args.dry_run)
                deleted_indexed_files += 1

    deleted_sweep_files = 0
    sweeps_dir = dataset_dir / "sweeps"
    if sweeps_dir.is_dir():
        for sensor_dir in sweeps_dir.iterdir():
            if not sensor_dir.is_dir():
                continue
            for path in sensor_dir.iterdir():
                if not path.is_file():
                    continue
                try:
                    timestamp = int(path.stem)
                except ValueError:
                    continue
                if timestamp < first_timestamp or timestamp > last_timestamp:
                    _unlink(path, args.dry_run)
                    deleted_sweep_files += 1

    removed_tracks = _trim_annotations(dataset_dir / "annotations.json", kept_timestamps, args.dry_run)
    _write_csv(csv_path, kept_rows, args.dry_run)

    mode = "Would keep" if args.dry_run else "Kept"
    print(f"{mode} {args.count} samples from {kept_timestamps[0]} to {kept_timestamps[-1]}")
    print(f"Indexed files removed: {deleted_indexed_files}")
    print(f"Sweep files removed: {deleted_sweep_files}")
    print(f"Annotation tracks removed: {removed_tracks}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
