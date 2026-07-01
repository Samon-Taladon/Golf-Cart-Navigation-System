import csv
import math

input_file = "path_smoothlog6.csv"
output_file = "path_smoothlog6_resampled_1m.csv"

spacing = 1.0

points = []

# ==========================================================
# LOAD PATH
# ==========================================================

with open(input_file, "r") as f:

    reader = csv.DictReader(f)

    for row in reader:

        x = float(row["x"])
        y = float(row["y"])

        points.append((x, y))

# ==========================================================
# RESAMPLE
# ==========================================================

new_points = [points[0]]

carry = 0.0
prev = points[0]

for i in range(1, len(points)):

    curr = points[i]

    dx = curr[0] - prev[0]
    dy = curr[1] - prev[1]

    seg_len = math.hypot(dx, dy)

    while carry + seg_len >= spacing:

        remain = spacing - carry

        ratio = remain / seg_len

        new_x = prev[0] + ratio * dx
        new_y = prev[1] + ratio * dy

        new_points.append((new_x, new_y))

        prev = (new_x, new_y)

        dx = curr[0] - prev[0]
        dy = curr[1] - prev[1]

        seg_len = math.hypot(dx, dy)

        carry = 0.0

    carry += seg_len

    prev = curr

# ==========================================================
# ADD LAST POINT
# ==========================================================

if new_points[-1] != points[-1]:

    new_points.append(points[-1])

# ==========================================================
# SAVE FILE
# ==========================================================

with open(output_file, "w", newline="") as f:

    writer = csv.writer(f)

    writer.writerow(["x", "y"])

    writer.writerows(new_points)

print(f"✅ Saved {len(new_points)} points")
print(f"📁 Output: {output_file}")