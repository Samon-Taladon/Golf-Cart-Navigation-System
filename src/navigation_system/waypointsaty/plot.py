"""
GPS Dashboard
=============
อ่านไฟล์ CSV รูปแบบ: lat, lon, speed, heading, fix_quality
แสดง:
  - แผนที่ lat/lon บน OpenStreetMap (folium) → gps_map.html
  - กราฟความเร็วและ heading (matplotlib) → gps_charts.png

ติดตั้ง dependencies:
    pip install folium matplotlib pandas

วิธีใช้:
    แก้ file_path ด้านล่างให้ตรงกับไฟล์ CSV ของคุณ แล้วรัน:
    python gps_dashboard.py
"""

import os
import sys
import webbrowser

import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import folium

# ── สีตาม fix_quality ─────────────────────────────────────────────────────────
FIX_COLOR = {
    4: "#22c55e",   # เขียว  → fix
    5: "#eab308",   # เหลือง → float
}
DEFAULT_COLOR = "#888888"   


# ── โหลด CSV ───────────────────────────────────────────────────────────────────
def load_data(csv_path: str) -> pd.DataFrame:
    # ==========================
    # โหลดข้อมูล CSV
    # ==========================
    file_path = "/home/inc/ros2_ws/src/navigation_system/waypointsaty/fastmode.csv"   
    df = pd.read_csv(file_path)
    # ==========================

    # ทำความสะอาดชื่อคอลัมน์
    df.columns = [c.strip().lower() for c in df.columns]

    required = {"lat", "lon", "speed", "heading", "fix_quality"}
    missing = required - set(df.columns)
    if missing:
        sys.exit(f"[error] ขาดคอลัมน์: {missing}\n"
                 f"  คอลัมน์ที่พบ: {list(df.columns)}")

    df = df.dropna(subset=list(required))
    df["fix_quality"] = df["fix_quality"].astype(int)

# ถ้าข้อมูลออก 1 Hz
    df["time_s"] = range(len(df))

    return df


# ── แผนที่ folium ─────────────────────────────────────────────────────────────
def build_map(df: pd.DataFrame, out_path: str) -> str:
    center_lat = df["lat"].mean()
    center_lon = df["lon"].mean()

    m = folium.Map(
        location=[center_lat, center_lon],
        zoom_start=17,
        tiles="OpenStreetMap",
    )

    coords = list(zip(df["lat"], df["lon"]))

    folium.PolyLine(
        coords,
        color="#9ca3af",
        weight=1.0,
        opacity=0.5
    ).add_to(m)

    for _, row in df.iterrows():
        folium.CircleMarker(
            location=[row["lat"], row["lon"]],
            radius=2,
            color="red",
            fill=True,
            fill_color="red",
            fill_opacity=0.9,
            weight=0,
            tooltip=(
                f"time: {row['time_s']} s<br>"
                f"lat: {row['lat']:.6f}<br>"
                f"lon: {row['lon']:.6f}<br>"
                f"speed: {row['speed']:.3f} kn<br>"
                f"heading: {row['heading']:.1f}°"
            ),
        ).add_to(m)

    m.save(out_path)
    return out_path


# ── กราฟ matplotlib ────────────────────────────────────────────────────────────
def build_charts(df: pd.DataFrame, out_path: str) -> str:

    time_s = df["time_s"]
    speeds = df["speed"]

    headings = ((df["heading"] + 180) % 360) - 180

    avg_sp = speeds.mean()
    min_sp = speeds.min()
    max_sp = speeds.max()

    fig = plt.figure(figsize=(11, 8), facecolor="#f9fafb")
    gs = gridspec.GridSpec(2, 1, hspace=0.48, figure=fig)

    # ==========================
    # Speed
    # ==========================
    ax1 = fig.add_subplot(gs[0])
    ax1.set_facecolor("#ffffff")

    ax1.plot(
        time_s,
        speeds,
        color="#2563eb",
        linewidth=0.5
    )

    ax1.fill_between(
        time_s,
        speeds,
        alpha=0.04,
        color="#2563eb"
    )

    ax1.set_ylabel("speed (knots)", fontsize=11)
    ax1.set_xlabel("time (s)", fontsize=10)
    ax1.set_title("Speed", fontsize=12, fontweight="bold")

    ax1.grid(True, linestyle="--", alpha=0.25)

    stat_text = (
        f"avg: {avg_sp:.4f}\n"
        f"min: {min_sp:.4f}\n"
        f"max: {max_sp:.4f}"
    )

    ax1.text(
        0.99,
        0.97,
        stat_text,
        transform=ax1.transAxes,
        verticalalignment="top",
        horizontalalignment="right",
        fontsize=9,
        family="monospace",
        bbox=dict(
            boxstyle="round,pad=0.4",
            facecolor="#eff6ff",
            edgecolor="#bfdbfe",
            alpha=0.9,
        ),
    )

    # ==========================
    # Yaw
    # ==========================
    ax2 = fig.add_subplot(gs[1])
    ax2.set_facecolor("#ffffff")

    ax2.plot(
        time_s,
        headings,
        color="#ea580c",
        linewidth=0.5
    )

    ax2.fill_between(
        time_s,
        headings,
        alpha=0.04,
        color="#ea580c"
    )

    ax2.set_ylim(-180, 180)
    ax2.set_yticks(range(-180, 181, 45))

    ax2.set_ylabel("yaw (°)", fontsize=11)
    ax2.set_xlabel("time (s)", fontsize=10)
    ax2.set_title("Yaw", fontsize=12, fontweight="bold")

    ax2.grid(True, linestyle="--", alpha=0.25)

    fig.suptitle(
        f"GPS Dashboard ({len(df)} points)",
        fontsize=13,
        fontweight="bold",
        y=0.99,
        color="#1e293b"
    )

    plt.savefig(
        out_path,
        dpi=200,
        bbox_inches="tight",
        facecolor=fig.get_facecolor()
    )

    plt.close()
    return out_path


# ── main ───────────────────────────────────────────────────────────────────────
def main():
    # ==========================
    # ตั้งค่าไฟล์และ output
    # ==========================
    file_path = "waypoints_clean.csv"  # เปลี่ยนเป็นชื่อไฟล์ของคุณ
    out_folder = "gps_output"          # โฟลเดอร์สำหรับ output
    # ==========================

    os.makedirs(out_folder, exist_ok=True)

    print("[1/3] โหลดข้อมูล...")
    df = load_data(file_path)
    print(f"      พบ {len(df)} จุด | fix_quality values: {sorted(df['fix_quality'].unique())}")

    map_path   = os.path.join(out_folder, "gps_map.html")
    chart_path = os.path.join(out_folder, "gps_charts.png")

    print("[2/3] สร้างแผนที่ OpenStreetMap →", map_path)
    build_map(df, map_path)

    print("[3/3] สร้างกราฟ speed & heading →", chart_path)
    build_charts(df, chart_path)

    print("\n✅ เสร็จแล้ว!")
    print(f"   แผนที่  : {os.path.abspath(map_path)}")
    print(f"   กราฟ    : {os.path.abspath(chart_path)}")

    webbrowser.open("file://" + os.path.abspath(map_path))
    img = plt.imread(chart_path)
    plt.figure(figsize=(11, 8))
    plt.imshow(img)
    plt.axis("off")
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()