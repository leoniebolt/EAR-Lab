import open3d as o3d
import numpy as np
import os
import math

# -------------------------------
# CONFIG
# -------------------------------
PCD_PATH = "FAST-LIO_map/GlobalMap.pcd"
OUTPUT_FILE = "wall_quality_report.txt"

DISTANCE_THRESHOLD = 0.05   # meters
MIN_PLANE_POINTS = 2000     
MAX_WALLS = 5               
Z_AXIS_THRESHOLD = 0.8      # Filter: If normal_z > 0.8, it's a floor (skip it)

# -------------------------------
# SETUP
# -------------------------------
print(f"Loading cloud: {PCD_PATH}")
pcd = o3d.io.read_point_cloud(PCD_PATH)

if len(pcd.points) == 0:
    print("Error: Point cloud is empty.")
    exit()

pcd = pcd.voxel_down_sample(voxel_size=0.02)
print(f"Loaded {len(pcd.points)} points.\n")

# Initialize lists to store stats for global average
all_rmse = []
all_psnr = []

# Initialize Report File
with open(OUTPUT_FILE, "w") as f:
    f.write("========================================\n")
    f.write("      WALL QUALITY & PSNR REPORT        \n")
    f.write("========================================\n\n")

remaining_cloud = pcd
walls_found = 0

# -------------------------------
# MAIN LOOP
# -------------------------------
while walls_found < MAX_WALLS:
    if len(remaining_cloud.points) < MIN_PLANE_POINTS:
        print("Not enough points left to find more walls.")
        break

    # 1. Detect Plane
    plane_model, inliers = remaining_cloud.segment_plane(
        distance_threshold=DISTANCE_THRESHOLD,
        ransac_n=3,
        num_iterations=1000
    )

    if len(inliers) < MIN_PLANE_POINTS:
        break

    [a, b, c, d] = plane_model
    
    # 2. Check Orientation (Filter out floors)
    # If Z-component of normal is high, it's a floor/ceiling
    if abs(c) > Z_AXIS_THRESHOLD:
        remaining_cloud = remaining_cloud.select_by_index(inliers, invert=True)
        continue

    # 3. Process Wall
    walls_found += 1
    
    # Extract points
    wall_cloud = remaining_cloud.select_by_index(inliers)
    wall_cloud.paint_uniform_color([1.0, 0.0, 0.0])  # Red Wall
    
    # Visualization Background (Dark Grey)
    temp_remaining = remaining_cloud.select_by_index(inliers, invert=True)
    temp_remaining.paint_uniform_color([0.1, 0.1, 0.1]) 

    # 4. Calculate RMSE
    pts = np.asarray(wall_cloud.points)
    distances = np.abs(a * pts[:, 0] + b * pts[:, 1] + c * pts[:, 2] + d)
    distances /= np.sqrt(a * a + b * b + c * c)
    rmse = np.sqrt(np.mean(distances ** 2))

    # 5. Calculate PSNR
    bbox = wall_cloud.get_axis_aligned_bounding_box()
    wall_diagonal = np.linalg.norm(bbox.get_max_bound() - bbox.get_min_bound())
    
    if rmse > 0:
        psnr = 20 * math.log10(wall_diagonal / rmse)
    else:
        psnr = 0.0

    # Store stats for global average
    all_rmse.append(rmse)
    all_psnr.append(psnr)

    print(f"[WALL #{walls_found}] Points: {len(pts)} | RMSE: {rmse:.4f}m | PSNR: {psnr:.2f} dB")

    # 6. Log Individual Wall to file
    with open(OUTPUT_FILE, "a") as f:
        f.write(f"Wall #{walls_found}\n")
        f.write(f"  Points: {len(pts)}\n")
        f.write(f"  RMSE:   {rmse:.4f} m\n")
        f.write(f"  PSNR:   {psnr:.2f} dB\n")
        f.write(f"  Size:   {wall_diagonal:.2f} m (diagonal)\n")
        f.write("-" * 30 + "\n")

    # 7. VISUALIZE
    print(f"   -> Opening Window... (Wall is RED). Close window to continue.")
    o3d.visualization.draw_geometries(
        [wall_cloud, temp_remaining],
        window_name=f"Wall #{walls_found} | RMSE: {rmse:.3f}m | PSNR: {psnr:.1f}dB",
        width=1024, height=768
    )

    # 8. Clean up for next iteration
    remaining_cloud = remaining_cloud.select_by_index(inliers, invert=True)

# -------------------------------
# GLOBAL EVALUATION
# -------------------------------
print("\nCalculating Global Metrics...")

if len(all_rmse) > 0:
    avg_rmse = np.mean(all_rmse)
    avg_psnr = np.mean(all_psnr)

    # Print to Console
    print("\n============================")
    print("GLOBAL MAP QUALITY")
    print("============================")
    print(f"Walls Analyzed: {walls_found}")
    print(f"Average RMSE:   {avg_rmse:.4f} m")
    print(f"Average PSNR:   {avg_psnr:.2f} dB")
    print(f"Report saved:   {OUTPUT_FILE}")

    # Write to File
    with open(OUTPUT_FILE, "a") as f:
        f.write("\n============================\n")
        f.write("GLOBAL EVALUATION (AVERAGE)\n")
        f.write("============================\n")
        f.write(f"Walls Analyzed: {walls_found}\n")
        f.write(f"Average RMSE:   {avg_rmse:.4f} m\n")
        f.write(f"Average PSNR:   {avg_psnr:.2f} dB\n")
else:
    print("No walls were detected, so no global stats could be calculated.")
    with open(OUTPUT_FILE, "a") as f:
        f.write("\nNo valid walls detected.\n")