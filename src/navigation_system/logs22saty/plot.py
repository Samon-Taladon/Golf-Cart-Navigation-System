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
    file_path = "/home/inc/ros2_ws/src/navigation_system/logs22saty/waypoints_clean copy.csv"   
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
    df["index_no"] = range(1, len(df) + 1)
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

    # วาดเส้นทาง (เทา บาง)
    coords = list(zip(df["lat"], df["lon"]))
    folium.PolyLine(coords, color="#9ca3af", weight=1.5, opacity=0.6).add_to(m)

    # วาดจุด
    for _, row in df.iterrows():
        color = FIX_COLOR.get(row["fix_quality"], DEFAULT_COLOR)
        folium.CircleMarker(
            location=[row["lat"], row["lon"]],
            radius=1.5,
            color=color,
            fill=True,
            fill_color=color,
            fill_opacity=0.9,
            weight=0,
            tooltip=(
                f"#{int(row['index_no'])}  fix_quality: {row['fix_quality']}<br>"
                f"lat: {row['lat']:.6f}  lon: {row['lon']:.6f}<br>"
                f"speed: {row['speed']:.4f} kn  heading: {row['heading']:.1f}°"
            ),
        ).add_to(m)

    # Legend (HTML overlay)
    legend_html = """
    <div style="
        position: fixed; top: 14px; right: 14px; z-index: 9999;
        background: white; border: 1px solid #e5e7eb;
        border-radius: 8px; padding: 8px 14px;
        font-family: sans-serif; font-size: 12px; line-height: 1.8;
        box-shadow: 0 1px 4px rgba(0,0,0,.12);">
      <b style="font-size:13px;">fix_quality</b><br>
      <span style="display:inline-block;width:10px;height:10px;
            border-radius:50%;background:#22c55e;vertical-align:middle;
            margin-right:5px;"></span>4 = fix<br>
      <span style="display:inline-block;width:10px;height:10px;
            border-radius:50%;background:#eab308;vertical-align:middle;
            margin-right:5px;"></span>5 = float
    </div>
    """
    m.get_root().html.add_child(folium.Element(legend_html))

    m.save(out_path)
    return out_path


# ── กราฟ matplotlib ────────────────────────────────────────────────────────────
def build_charts(df: pd.DataFrame, out_path: str) -> str:
    idx = df["index_no"]
    speeds = df["speed"]
    # แปลง heading (0~360) -> yaw (-180~180)
    headings = ((df["heading"] + 180) % 360) - 180

    avg_sp = speeds.mean()
    min_sp = speeds.min()
    max_sp = speeds.max()

    fig = plt.figure(figsize=(11, 8), facecolor="#f9fafb")
    gs = gridspec.GridSpec(2, 1, hspace=0.48, figure=fig)

    # ─── กราฟความเร็ว ─────────────────────────────────────────────
    ax1 = fig.add_subplot(gs[0])
    ax1.set_facecolor("#ffffff")
    ax1.plot(idx, speeds, color="#3b82f6", linewidth=1.6,
             marker="o", markersize=3, markerfacecolor="#3b82f6", label="speed")
    ax1.fill_between(idx, speeds, alpha=0.08, color="#3b82f6")
    ax1.set_ylabel("speed (knots)", fontsize=11)
    ax1.set_xlabel("data point", fontsize=10)
    ax1.set_title("Speed", fontsize=12, fontweight="bold", pad=8)
    ax1.grid(True, linestyle="--", alpha=0.35)
    ax1.tick_params(labelsize=9)

    # stat box มุมบนขวา
    stat_text = (
        f"avg: {avg_sp:.4f}\n"
        f"min: {min_sp:.4f}\n"
        f"max: {max_sp:.4f}"
    )
    ax1.text(
        0.99, 0.97, stat_text,
        transform=ax1.transAxes,
        verticalalignment="top", horizontalalignment="right",
        fontsize=9, family="monospace",
        bbox=dict(boxstyle="round,pad=0.4", facecolor="#eff6ff",
                  edgecolor="#bfdbfe", alpha=0.9),
    )

    # ─── กราฟ Heading ────────────────────────────────────────────
    ax2 = fig.add_subplot(gs[1])
    ax2.set_facecolor("#ffffff")
    ax2.plot(idx, headings, color="#f97316", linewidth=1.6,
             marker="o", markersize=3, markerfacecolor="#f97316", label="heading")
    ax2.fill_between(idx, headings, alpha=0.08, color="#f97316")
    ax2.set_ylim(-180, 180)
    ax2.set_yticks(range(-180, 181, 45))
    ax2.set_ylabel("yaw (°)", fontsize=11)
    ax2.set_title("Yaw", fontsize=12, fontweight="bold", pad=8)
    ax2.grid(True, linestyle="--", alpha=0.35)
    ax2.tick_params(labelsize=9)

    fig.suptitle(
        f"GPS Dashboard  ({len(df)} points)",
        fontsize=13, fontweight="bold", y=0.99, color="#1e293b"
    )

    plt.savefig(out_path, dpi=150, bbox_inches="tight",
                facecolor=fig.get_facecolor())
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