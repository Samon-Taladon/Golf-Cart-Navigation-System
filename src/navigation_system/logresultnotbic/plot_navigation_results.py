#!/usr/bin/env python3
"""
Create five PNG figures from:
  1) navigation result CSV: x, y, speed, heading, fix_quality, steering_angle
  2) waypoint CSV: x, y

Outputs:
  1_xy_trajectory.png
  2_speed.png
  3_cte.png
  4_heading.png
  5_steering.png

Speed, CTE, Heading, and Steering are plotted as continuous status-colored lines
(no point markers):
  RTK Fix   = green
  RTK Float = yellow/gold
  Waypoints = black

Example:
  python3 plot_navigation_results.py \
      navigation_20260618_224458_natural_speed_like_file2(1).csv \
      path_smoothlog13_resampled_1m.csv \
      --dt 0.05 --outdir plots
"""

import argparse
from pathlib import Path
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.collections import LineCollection

GREEN = "green"
YELLOW = "#E5B700"
BLACK = "black"
GRAY = "0.65"
DPI = 300


def require_columns(df, required, filename):
    missing = [c for c in required if c not in df.columns]
    if missing:
        raise ValueError(
            f"{filename} is missing required columns: {missing}\n"
            f"Available columns: {list(df.columns)}"
        )


def signed_cte_to_path(px, py, wx, wy):
    """
    Signed cross-track error relative to the ordered waypoint polyline.

    Sign convention requested for this project:
      CTE < 0  -> vehicle is LEFT of the waypoint/path direction
      CTE > 0  -> vehicle is RIGHT of the waypoint/path direction

    The error is measured to the nearest *path segment* (not merely the
    nearest waypoint), which gives a more meaningful CTE for a continuous path.
    """
    points = np.column_stack((px, py))
    a = np.column_stack((wx[:-1], wy[:-1]))
    b = np.column_stack((wx[1:], wy[1:]))
    v = b - a
    vv = np.sum(v * v, axis=1)

    valid = vv > 1e-12
    a = a[valid]
    v = v[valid]
    vv = vv[valid]
    if len(a) == 0:
        raise ValueError("Waypoint path must contain at least one non-zero segment.")

    signed = np.empty(len(points), dtype=float)
    chunk = 500
    for i in range(0, len(points), chunk):
        p = points[i:i + chunk]
        ap = p[:, None, :] - a[None, :, :]
        t = np.sum(ap * v[None, :, :], axis=2) / vv[None, :]
        t = np.clip(t, 0.0, 1.0)
        proj = a[None, :, :] + t[:, :, None] * v[None, :, :]
        e = p[:, None, :] - proj
        d2 = np.sum(e * e, axis=2)
        idx = np.argmin(d2, axis=1)
        rows = np.arange(len(p))
        nearest_e = e[rows, idx]
        nearest_v = v[idx]
        dist = np.sqrt(d2[rows, idx])

        # cross(v, e) > 0 means LEFT of path direction.
        # User requested LEFT negative, RIGHT positive, hence the minus sign.
        cross = nearest_v[:, 0] * nearest_e[:, 1] - nearest_v[:, 1] * nearest_e[:, 0]
        side = np.sign(cross)
        signed[i:i + len(p)] = -side * dist

    return signed


def _status_colors(quality, fix_code, float_codes):
    """Return one color per sample according to GNSS quality."""
    float_codes = set(int(v) for v in float_codes)
    colors = np.full(len(quality), GRAY, dtype=object)
    colors[np.asarray(quality) == int(fix_code)] = GREEN
    colors[np.isin(quality, list(float_codes))] = YELLOW
    return colors


def status_line(ax, x, y, quality, fix_code, float_codes, linewidth=1.2):
    """
    Draw one continuous status-colored line using LineCollection.

    Unlike the old NaN-mask method, this colors each segment between adjacent
    samples. Therefore the curve does NOT visually break when GNSS status
    switches between RTK Fix and RTK Float.

    Other GNSS states are shown in gray rather than discarded.
    """
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    q = np.asarray(quality)

    if len(x) < 2:
        return

    finite = np.isfinite(x) & np.isfinite(y)
    p0 = np.column_stack((x[:-1], y[:-1]))
    p1 = np.column_stack((x[1:], y[1:]))
    valid = finite[:-1] & finite[1:]
    segments = np.stack((p0[valid], p1[valid]), axis=1)

    sample_colors = _status_colors(q, fix_code, float_codes)
    # Use the status of the later sample for each adjacent line segment.
    seg_colors = sample_colors[1:][valid]

    lc = LineCollection(
        segments,
        colors=seg_colors.tolist(),
        linewidths=linewidth,
        capstyle="round",
        joinstyle="round",
        zorder=2,
    )
    ax.add_collection(lc)
    ax.update_datalim(np.column_stack((x[finite], y[finite])))
    ax.autoscale_view()


def trajectory_status_line(ax, x, y, quality, fix_code, float_codes, linewidth=1.2):
    """Continuous status-colored XY trajectory with no gaps at status changes."""
    status_line(ax, x, y, quality, fix_code, float_codes, linewidth=linewidth)


def metric_box(ax, lines, loc="upper right", fontsize=9):
    text = "\n".join(lines)
    positions = {
        "upper right": (0.98, 0.97, "right", "top"),
        "upper left": (0.02, 0.97, "left", "top"),
        "lower right": (0.98, 0.03, "right", "bottom"),
        "lower left": (0.02, 0.03, "left", "bottom"),
    }
    x, y, ha, va = positions[loc]
    ax.text(
        x, y, text,
        transform=ax.transAxes,
        ha=ha,
        va=va,
        fontsize=fontsize,
        bbox=dict(
            boxstyle="square,pad=0.25",
            facecolor="white",
            edgecolor="0.65",
            alpha=0.92,
        ),
    )


def style_axis(ax):
    ax.grid(True, color="0.65", linewidth=0.8, alpha=0.75)
    ax.tick_params(labelsize=10)
    for spine in ax.spines.values():
        spine.set_linewidth(0.9)


def add_status_legend(ax, include_waypoints=False, loc="upper left"):

    handles = [
        Line2D([0], [0], color=GREEN, lw=2.0, label="RTK Fix"),
        Line2D([0], [0], color=YELLOW, lw=2.0, label="RTK Float"),
    ]
    if include_waypoints:
        handles.append(Line2D([0], [0], color=BLACK, lw=1.8, label="Waypoints"))
    ax.legend(handles=handles, loc=loc, frameon=True, fontsize=8, handlelength=2.2, borderpad=0.35, labelspacing=0.25)


def main():
    parser = argparse.ArgumentParser(
        description="Plot golf-cart navigation experiment results."
    )
    parser.add_argument("result_csv", help="Navigation result CSV")
    parser.add_argument("waypoint_csv", help="Waypoint CSV containing x,y")
    parser.add_argument("--outdir", default="navigation_plots", help="Output directory")
    parser.add_argument(
        "--dt", type=float, default=0.05,
        help="Sampling period when CSV has no time column (default: 0.05 s)"
    )
    parser.add_argument(
        "--fix-code", type=int, default=4,
        help="fix_quality value representing RTK Fix (default: 4)"
    )
    parser.add_argument(
        "--float-codes", type=int, nargs="+", default=[5],
        help="fix_quality value(s) treated as RTK Float (default: 5). "
             "Example for Float + DGPS: --float-codes 5 2"
    )
    parser.add_argument(
        "--time-column", default=None,
        help="Optional time column. If omitted, sample_index * dt is used."
    )
    args = parser.parse_args()

    result_path = Path(args.result_csv)
    waypoint_path = Path(args.waypoint_csv)
    outdir = Path(args.outdir)
    outdir.mkdir(parents=True, exist_ok=True)

    df = pd.read_csv(result_path)
    wp = pd.read_csv(waypoint_path)

    require_columns(
        df,
        ["x", "y", "speed", "heading", "fix_quality", "steering_angle"],
        result_path.name,
    )
    require_columns(wp, ["x", "y"], waypoint_path.name)

    df = df.dropna(
        subset=["x", "y", "speed", "heading", "fix_quality", "steering_angle"]
    ).reset_index(drop=True)
    wp = wp.dropna(subset=["x", "y"]).reset_index(drop=True)

    if len(df) == 0 or len(wp) == 0:
        raise ValueError("Input CSV contains no usable rows after removing NaN values.")

    x = df["x"].to_numpy(float)
    y = df["y"].to_numpy(float)
    speed = df["speed"].to_numpy(float)
    heading = df["heading"].to_numpy(float)
    steering = df["steering_angle"].to_numpy(float)
    quality = df["fix_quality"].to_numpy(int)

    wx = wp["x"].to_numpy(float)
    wy = wp["y"].to_numpy(float)

    if args.time_column:
        if args.time_column not in df.columns:
            raise ValueError(f"Time column '{args.time_column}' not found in result CSV.")
        time = df[args.time_column].to_numpy(float)
        time = time - time[0]
    else:
        time = np.arange(len(df), dtype=float) * args.dt

    fix_mask = quality == args.fix_code
    float_mask = np.isin(quality, args.float_codes)

    cte = signed_cte_to_path(x, y, wx, wy)

    mean_speed = float(np.mean(speed))
    max_speed = float(np.max(speed))
    mean_cte = float(np.mean(cte))
    max_cte = float(np.max(np.abs(cte)))
    rmse_cte = float(np.sqrt(np.mean(cte ** 2)))

    # 1) XY trajectory
    fig, ax = plt.subplots(figsize=(8, 7))
    ax.plot(wx, wy, color=BLACK, linewidth=1.8, label="Waypoints", zorder=1)
    trajectory_status_line(ax, x, y, quality, args.fix_code, args.float_codes, linewidth=1.2)
    add_status_legend(ax, include_waypoints=True, loc="upper right")
    ax.set_title("XY Trajectory", fontsize=14, fontweight="bold")
    ax.set_xlabel("X position (m)")
    ax.set_ylabel("Y position (m)")
    ax.axis("equal")
    style_axis(ax)
    fig.tight_layout()
    fig.savefig(outdir / "1_xy_trajectory.png", dpi=DPI, bbox_inches="tight")
    plt.close(fig)

    # 2) Speed - lines only
    fig, ax = plt.subplots(figsize=(10, 5.5))
    status_line(ax, time, speed, quality, args.fix_code, args.float_codes, linewidth=1.2)
    add_status_legend(ax, loc="upper left")
    metric_box(ax, [
        f"Mean = {mean_speed:.3f} m/s",
        f"Max  = {max_speed:.3f} m/s",
    ], loc="upper right", fontsize=9)
    ax.set_title("Speed", fontsize=14, fontweight="bold")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Speed (m/s)")
    style_axis(ax)
    fig.tight_layout()
    fig.savefig(outdir / "2_speed.png", dpi=DPI, bbox_inches="tight")
    plt.close(fig)

    # 3) CTE - lines only
    fig, ax = plt.subplots(figsize=(10, 5.5))
    status_line(ax, time, cte, quality, args.fix_code, args.float_codes, linewidth=1.2)
    add_status_legend(ax, loc="upper left")
    metric_box(ax, [
        f"Mean = {mean_cte:.3f} m",
        f"Max |CTE| = {max_cte:.3f} m",
        f"RMSE = {rmse_cte:.3f} m",
    ], loc="upper right", fontsize=9)
    ax.set_title("Cross-Track Error (CTE)", fontsize=14, fontweight="bold")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Signed CTE (m)")
    ax.axhline(0.0, color="0.25", linewidth=1.0, linestyle="--", alpha=0.9)
    style_axis(ax)
    fig.tight_layout()
    fig.savefig(outdir / "3_cte.png", dpi=DPI, bbox_inches="tight")
    plt.close(fig)

    # 4) Heading - lines only
    fig, ax = plt.subplots(figsize=(10, 5.5))
    status_line(ax, time, heading, quality, args.fix_code, args.float_codes, linewidth=1.2)
    add_status_legend(ax, loc="upper left")
    ax.set_title("Heading", fontsize=14, fontweight="bold")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Heading (deg)")
    style_axis(ax)
    fig.tight_layout()
    fig.savefig(outdir / "4_heading.png", dpi=DPI, bbox_inches="tight")
    plt.close(fig)

    # 5) Steering - lines only
    fig, ax = plt.subplots(figsize=(10, 5.5))
    status_line(ax, time, steering, quality, args.fix_code, args.float_codes, linewidth=1.2)
    add_status_legend(ax, loc="upper left")
    ax.axhline(0.0, color="black", linewidth=0.8, linestyle="--", alpha=0.6)
    ax.set_title("Steering Angle", fontsize=14, fontweight="bold")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Steering Angle (rad)")
    style_axis(ax)
    fig.tight_layout()
    fig.savefig(outdir / "5_steering.png", dpi=DPI, bbox_inches="tight")
    plt.close(fig)

    names = [
        "1_xy_trajectory.png",
        "2_speed.png",
        "3_cte.png",
        "4_heading.png",
        "5_steering.png",
    ]

    print("Created:")
    for name in names:
        print(f"  {outdir / name}")

    print("\nMetrics:")
    print(f"  Mean Speed = {mean_speed:.6f} m/s")
    print(f"  Max Speed  = {max_speed:.6f} m/s")
    print(f"  Mean CTE   = {mean_cte:.6f} m")
    print(f"  Max CTE    = {max_cte:.6f} m")
    print(f"  RMSE CTE   = {rmse_cte:.6f} m")


if __name__ == "__main__":
    main()
