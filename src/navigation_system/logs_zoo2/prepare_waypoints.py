#!/usr/bin/env python3
"""Prepare logs_zoo2 waypoints for enhanced_gnss_publisher.py and purepursuit.py.

Input:
    waypoints_clean.csv with columns: lat,lon,speed,heading,fix_quality

Outputs:
    waypoints_utmlogs_zoo2.csv              lat/lon converted to UTM x/y
    path_smoothlogs_zoo2.csv                smoothed and distance-filtered x/y path
    path_smoothlogs_zoo2_resampled_1m.csv   purepursuit-compatible x,y path

The GNSS publisher publishes absolute UTM Zone 47N coordinates on /odom, so the
Pure Pursuit path must use the same absolute UTM coordinate frame.
"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
from typing import Iterable

from pyproj import Transformer


DEFAULT_WINDOW_SIZE = 5
DEFAULT_MIN_POINT_DISTANCE_M = 0.5
DEFAULT_RESAMPLE_SPACING_M = 1.0


def parse_args() -> argparse.Namespace:
    script_dir = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(
        description="Convert logs_zoo2 lat/lon waypoints to UTM and Pure Pursuit path files."
    )
    parser.add_argument(
        "--input",
        type=Path,
        default=script_dir / "waypoints_clean.csv",
        help="Input CSV with lat,lon columns.",
    )
    parser.add_argument(
        "--utm-output",
        type=Path,
        default=script_dir / "waypoints_utmlogs_zoo2.csv",
        help="Output CSV containing x,y plus original waypoint metadata.",
    )
    parser.add_argument(
        "--smooth-output",
        type=Path,
        default=script_dir / "path_smoothlogs_zoo2.csv",
        help="Output smoothed path CSV with x,y columns.",
    )
    parser.add_argument(
        "--resampled-output",
        type=Path,
        default=script_dir / "path_smoothlogs_zoo2_resampled_1m.csv",
        help="Output resampled path CSV with x,y columns for purepursuit.py.",
    )
    parser.add_argument(
        "--window-size",
        type=int,
        default=DEFAULT_WINDOW_SIZE,
        help="Centered moving average window size for smoothing.",
    )
    parser.add_argument(
        "--min-point-distance",
        type=float,
        default=DEFAULT_MIN_POINT_DISTANCE_M,
        help="Minimum distance between smoothed path points before resampling.",
    )
    parser.add_argument(
        "--spacing",
        type=float,
        default=DEFAULT_RESAMPLE_SPACING_M,
        help="Distance between final resampled path points.",
    )
    return parser.parse_args()


def read_waypoints(input_path: Path) -> list[dict[str, str]]:
    with input_path.open("r", newline="", encoding="utf-8") as file:
        reader = csv.DictReader(file)
        if reader.fieldnames is None:
            raise ValueError(f"{input_path} has no CSV header")

        required = {"lat", "lon"}
        missing = required.difference(reader.fieldnames)
        if missing:
            raise ValueError(
                f"{input_path} missing required columns: {', '.join(sorted(missing))}"
            )

        rows = list(reader)

    if len(rows) < 2:
        raise ValueError("Need at least two waypoint rows")

    return rows


def convert_to_utm(rows: list[dict[str, str]]) -> list[dict[str, str]]:
    transformer = Transformer.from_crs("epsg:4326", "epsg:32647", always_xy=True)
    converted: list[dict[str, str]] = []

    for row_number, row in enumerate(rows, start=2):
        try:
            lat = float(row["lat"])
            lon = float(row["lon"])
        except ValueError as exc:
            raise ValueError(f"Invalid lat/lon at CSV row {row_number}: {row}") from exc

        x, y = transformer.transform(lon, lat)
        output_row = {
            "x": f"{x:.9f}",
            "y": f"{y:.9f}",
        }

        for key, value in row.items():
            if key not in {"lat", "lon"}:
                output_row[key] = value

        converted.append(output_row)

    return converted


def write_dict_csv(path: Path, rows: list[dict[str, str]], fieldnames: list[str]) -> None:
    with path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def moving_average(values: list[float], window_size: int) -> list[float]:
    if window_size < 1:
        raise ValueError("window-size must be >= 1")

    half_window = window_size // 2
    averaged: list[float] = []

    for index, value in enumerate(values):
        start = index - half_window
        end = start + window_size
        if start < 0 or end > len(values):
            averaged.append(value)
            continue

        window = values[start:end]
        averaged.append(sum(window) / len(window))

    return averaged


def distance_filter(
    points: Iterable[tuple[float, float]], min_distance: float
) -> list[tuple[float, float]]:
    if min_distance <= 0.0:
        raise ValueError("min-point-distance must be > 0")

    filtered: list[tuple[float, float]] = []
    last_point: tuple[float, float] | None = None

    for point in points:
        if last_point is None:
            filtered.append(point)
            last_point = point
            continue

        if math.hypot(point[0] - last_point[0], point[1] - last_point[1]) >= min_distance:
            filtered.append(point)
            last_point = point

    return filtered


def resample_path(
    points: list[tuple[float, float]], spacing: float
) -> list[tuple[float, float]]:
    if spacing <= 0.0:
        raise ValueError("spacing must be > 0")
    if len(points) < 2:
        raise ValueError("Need at least two points to resample")

    new_points = [points[0]]
    carry = 0.0
    prev = points[0]

    for current in points[1:]:
        dx = current[0] - prev[0]
        dy = current[1] - prev[1]
        segment_length = math.hypot(dx, dy)

        while segment_length > 0.0 and carry + segment_length >= spacing:
            remain = spacing - carry
            ratio = remain / segment_length
            new_point = (prev[0] + ratio * dx, prev[1] + ratio * dy)
            new_points.append(new_point)

            prev = new_point
            dx = current[0] - prev[0]
            dy = current[1] - prev[1]
            segment_length = math.hypot(dx, dy)
            carry = 0.0

        carry += segment_length
        prev = current

    if new_points[-1] != points[-1]:
        new_points.append(points[-1])

    return new_points


def write_xy_csv(path: Path, points: list[tuple[float, float]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.writer(file)
        writer.writerow(["x", "y"])
        writer.writerows((f"{x:.9f}", f"{y:.9f}") for x, y in points)


def main() -> None:
    args = parse_args()
    rows = read_waypoints(args.input)
    utm_rows = convert_to_utm(rows)

    metadata_columns = [name for name in rows[0].keys() if name not in {"lat", "lon"}]
    write_dict_csv(args.utm_output, utm_rows, ["x", "y", *metadata_columns])

    x_values = [float(row["x"]) for row in utm_rows]
    y_values = [float(row["y"]) for row in utm_rows]
    smooth_points = list(
        zip(
            moving_average(x_values, args.window_size),
            moving_average(y_values, args.window_size),
        )
    )
    filtered_points = distance_filter(smooth_points, args.min_point_distance)
    resampled_points = resample_path(filtered_points, args.spacing)

    write_xy_csv(args.smooth_output, filtered_points)
    write_xy_csv(args.resampled_output, resampled_points)

    print(f"Input waypoints: {len(rows)}")
    print(f"UTM output: {args.utm_output} ({len(utm_rows)} rows)")
    print(f"Smoothed output: {args.smooth_output} ({len(filtered_points)} points)")
    print(f"Pure Pursuit output: {args.resampled_output} ({len(resampled_points)} points)")


if __name__ == "__main__":
    main()
