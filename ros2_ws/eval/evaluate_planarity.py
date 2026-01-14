import open3d as o3d
import numpy as np
import math
import copy

# -------------------------------
# CONFIGURATION
# -------------------------------
PCD_PATH = "LVI-SAM_map/GlobalMap.pcd"
OUTPUT_FILE = "wall_quality_inclusive.txt"

# DETECTION SETTINGS (Finding the wall)
DETECT_THRESHOLD = 0.05     # RANSAC threshold (tight)
MIN_PLANE_POINTS = 2000     
MAX_WALLS = 20               
Z_AXIS_THRESHOLD = 0.8      # Filter out floors

# EVALUATION SETTINGS (Grading the wall)
# We will look for points this far away from the center of the wall
# 0.60 meters = +/- 30cm on each side
EVALUATION_THICKNESS = 0.60 

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

# Global Stats
all_rmse = []
all_psnr = []

with open(OUTPUT_FILE, "w") as f:
    f.write("========================================\n")
    f.write("   INCLUSIVE WALL QUALITY REPORT        \n")
    f.write(f"   (Evaluating thickness: {EVALUATION_THICKNESS}m)\n")
    f.write("========================================\n\n")

remaining_cloud = pcd
walls_found = 0

# -------------------------------
# MAIN LOOP
# -------------------------------
while walls_found < MAX_WALLS:
    if len(remaining_cloud.points) < MIN_PLANE_POINTS:
        print("Not enough points left.")
        break

    # 1. DETECT (Find the ideal location using RANSAC)
    plane_model, inliers = remaining_cloud.segment_plane(
        distance_threshold=DETECT_THRESHOLD,
        ransac_n=3,
        num_iterations=1000
    )

    if len(inliers) < MIN_PLANE_POINTS:
        break

    [a, b, c, d] = plane_model
    
    # Check if Floor
    if abs(c) > Z_AXIS_THRESHOLD:
        remaining_cloud = remaining_cloud.select_by_index(inliers, invert=True)
        continue

    walls_found += 1
    
    # 2. CREATE EXPANDED BOUNDING BOX
    # Extract the "perfect" points just to get the orientation/center
    core_wall = remaining_cloud.select_by_index(inliers)
    
    # Calculate Oriented Bounding Box (OBB) of the core wall
    obb = core_wall.get_oriented_bounding_box()
    
    # Get current center, rotation (R), and size (extent)
    center = obb.center
    R = obb.R
    extent = obb.extent # [width, height, depth]
    
    # Find which dimension is the "thickness" (the smallest one)
    min_axis = np.argmin(extent)
    
    # Force the thickness to be our EVALUATION_THICKNESS
    new_extent = copy.deepcopy(extent)
    new_extent[min_axis] = EVALUATION_THICKNESS
    
    # Create the new Expanded Box
    expanded_obb = o3d.geometry.OrientedBoundingBox(center, R, new_extent)
    expanded_obb.color = (1, 0, 0) # Red wireframe

    # 3. CROP (Capture ALL points in that box, including noise)
    # We crop from 'remaining_cloud' to catch the drift
    evaluation_cloud = remaining_cloud.crop(expanded_obb)
    
    # Color them GREEN for visualization
    evaluation_cloud.paint_uniform_color([0.0, 1.0, 0.0]) 

    # 4. CALCULATE METRICS (Using the extended cloud)
    pts = np.asarray(evaluation_cloud.points)
    
    # Calculate distance of ALL these points to the Ideal Plane
    distances = np.abs(a * pts[:, 0] + b * pts[:, 1] + c * pts[:, 2] + d)
    distances /= np.sqrt(a * a + b * b + c * c)
    
    rmse = np.sqrt(np.mean(distances ** 2))
    
    # Calculate PSNR
    wall_diagonal = np.linalg.norm(new_extent) # Use box size as reference
    if rmse > 0:
        psnr = 20 * math.log10(wall_diagonal / rmse)
    else:
        psnr = 0.0

    all_rmse.append(rmse)
    all_psnr.append(psnr)

    print(f"[WALL #{walls_found}]")
    print(f"  Points captured: {len(pts)} (Core + Noise)")
    print(f"  RMSE: {rmse:.4f} m")
    print(f"  PSNR: {psnr:.2f} dB")

    # 5. LOGGING
    with open(OUTPUT_FILE, "a") as f:
        f.write(f"Wall #{walls_found}\n")
        f.write(f"  Evaluation Thickness: {EVALUATION_THICKNESS} m\n")
        f.write(f"  Points: {len(pts)}\n")
        f.write(f"  RMSE:   {rmse:.4f} m\n")
        f.write(f"  PSNR:   {psnr:.2f} dB\n")
        f.write("-" * 30 + "\n")

    # 6. VISUALIZATION
    # Background points (Dark Grey)
    # We remove the *evaluation points* from the background to avoid overlap
    # Note: crop() returns a new cloud, so we need a clever way to visualize the rest.
    # Simple way: Just paint the whole remaining cloud grey, then draw the green on top.
    viz_background = copy.deepcopy(remaining_cloud)
    viz_background.paint_uniform_color([0.1, 0.1, 0.1])
    
    print("  -> Opening Window... (Green = Data | Red Box = Search Area)")
    # o3d.visualization.draw_geometries(
    #     [viz_background, evaluation_cloud, expanded_obb],
    #     window_name=f"Wall #{walls_found} Inclusive Eval",
    #     width=1024, height=768
    # )

    # 7. CLEANUP
    # Remove the *original inliers* (the core wall) so we don't find it again.
    remaining_cloud = remaining_cloud.select_by_index(inliers, invert=True)

# -------------------------------
# GLOBAL EVALUATION
# -------------------------------
if len(all_rmse) > 0:
    avg_rmse = np.mean(all_rmse)
    avg_psnr = np.mean(all_psnr)
    print("\n============================")
    print(f"Global RMSE: {avg_rmse:.4f} m")
    print(f"Global PSNR: {avg_psnr:.2f} dB")
    
    with open(OUTPUT_FILE, "a") as f:
        f.write("\nGLOBAL AVERAGES\n")
        f.write(f"RMSE: {avg_rmse:.4f} m\n")
        f.write(f"PSNR: {avg_psnr:.2f} dB\n")
