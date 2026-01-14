import matplotlib.pyplot as plt
import numpy as np
import re
import os

# -------------------------------
# CONFIGURATION
# -------------------------------
# Map friendly names to file paths
FILES = {
    "FAST-LIO":   "/home/ubuntu/ros2_ws/eval/FAST-LIO_map/FAST-LIO_map.txt",
    "LILIOC-SAM": "/home/ubuntu/ros2_ws/eval/LILIOC-SAM_map/LILIOC-SAM_map.txt",
    "LIO-SAM":    "/home/ubuntu/ros2_ws/eval/LIO-SAM_map/LIO-SAM_map.txt",
    "LVI-SAM":    "/home/ubuntu/ros2_ws/eval/LVI-SAM_map/LVI-SAM_map.txt"
}

def parse_report(filepath):
    """
    Parses a wall quality report file.
    Returns:
        global_stats: dict {'rmse': float, 'psnr': float}
        wall_stats:   dict {'rmse': [], 'psnr': []} containing lists of values
    """
    data = {
        "global": {"rmse": None, "psnr": None},
        "walls": {"rmse": [], "psnr": []}
    }
    
    if not os.path.exists(filepath):
        print(f"Warning: File not found: {filepath}")
        return None

    with open(filepath, 'r') as f:
        content = f.read()

    # 1. Extract Global Averages
    global_rmse_match = re.search(r"GLOBAL AVERAGES.*?RMSE:\s*([0-9.]+)\s*m", content, re.DOTALL)
    global_psnr_match = re.search(r"GLOBAL AVERAGES.*?PSNR:\s*([0-9.]+)\s*dB", content, re.DOTALL)

    if global_rmse_match:
        data["global"]["rmse"] = float(global_rmse_match.group(1))
    if global_psnr_match:
        data["global"]["psnr"] = float(global_psnr_match.group(1))

    # 2. Extract Individual Wall Data
    wall_sections = content.split("Wall #")[1:] # Skip preamble
    
    for section in wall_sections:
        rmse_m = re.search(r"RMSE:\s*([0-9.]+)\s*m", section)
        psnr_m = re.search(r"PSNR:\s*([0-9.]+)\s*dB", section)
        
        if rmse_m and psnr_m:
            data["walls"]["rmse"].append(float(rmse_m.group(1)))
            data["walls"]["psnr"].append(float(psnr_m.group(1)))

    return data

def plot_results(results):
    names = list(results.keys())
    
    # Prepare data for Global Averages Bar Chart
    avg_rmses = [results[n]["global"]["rmse"] for n in names]
    avg_psnrs = [results[n]["global"]["psnr"] for n in names]
    
    # Prepare data for Box Plots (Lists of lists)
    dist_rmse = [results[n]["walls"]["rmse"] for n in names]
    dist_psnr = [results[n]["walls"]["psnr"] for n in names]

    # Setup Plot: 2x2 Grid
    fig, axs = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('SLAM Algorithm Map Quality Comparison', fontsize=16)

    # ---------------------------
    # 1. Global RMSE Comparison (Bar)
    # ---------------------------
    bars = axs[0, 0].bar(names, avg_rmses, color=['#ff9999', '#66b3ff', '#99ff99', '#ffcc99'])
    axs[0, 0].set_title('Global Average RMSE (Lower is Better)')
    axs[0, 0].set_ylabel('RMSE (m)')
    axs[0, 0].grid(axis='y', linestyle='--', alpha=0.7)
    
    # Add value labels on top of bars
    for bar in bars:
        height = bar.get_height()
        if height is not None:
            axs[0, 0].text(bar.get_x() + bar.get_width()/2., height,
                        f'{height:.4f}', ha='center', va='bottom')

    # ---------------------------
    # 2. Global PSNR Comparison (Bar)
    # ---------------------------
    bars = axs[0, 1].bar(names, avg_psnrs, color=['#ff9999', '#66b3ff', '#99ff99', '#ffcc99'])
    axs[0, 1].set_title('Global Average PSNR (Higher is Better)')
    axs[0, 1].set_ylabel('PSNR (dB)')
    axs[0, 1].grid(axis='y', linestyle='--', alpha=0.7)

    for bar in bars:
        height = bar.get_height()
        if height is not None:
            axs[0, 1].text(bar.get_x() + bar.get_width()/2., height,
                        f'{height:.2f}', ha='center', va='bottom')

    # ---------------------------
    # 3. RMSE Distribution (Box Plot)
    # ---------------------------
    # FIXED ARGUMENT HERE: changed 'tick_labels' to 'labels'
    axs[1, 0].boxplot(dist_rmse, labels=names, patch_artist=True, 
                      boxprops=dict(facecolor='#ff9999', color='black'),
                      medianprops=dict(color='black'))
    axs[1, 0].set_title('RMSE Distribution (Wall Consistency)')
    axs[1, 0].set_ylabel('RMSE (m)')
    axs[1, 0].grid(axis='y', linestyle='--', alpha=0.7)

    # ---------------------------
    # 4. PSNR Distribution (Box Plot)
    # ---------------------------
    # FIXED ARGUMENT HERE: changed 'tick_labels' to 'labels'
    axs[1, 1].boxplot(dist_psnr, labels=names, patch_artist=True,
                      boxprops=dict(facecolor='#66b3ff', color='black'),
                      medianprops=dict(color='black'))
    axs[1, 1].set_title('PSNR Distribution (Signal Quality)')
    axs[1, 1].set_ylabel('PSNR (dB)')
    axs[1, 1].grid(axis='y', linestyle='--', alpha=0.7)

    plt.tight_layout(rect=[0, 0.03, 1, 0.95]) # Adjust for suptitle
    
    # Save and Show
    output_path = "slam_comparison_plot.png"
    plt.savefig(output_path)
    print(f"Plot saved to {output_path}")
    # plt.show() # Commented out to prevent errors if no display is available

# -------------------------------
# MAIN EXECUTION
# -------------------------------
if __name__ == "__main__":
    results = {}
    
    print("Parsing reports...")
    for name, path in FILES.items():
        data = parse_report(path)
        if data and data["global"]["rmse"] is not None:
            results[name] = data
            print(f"  Loaded {name}: Avg RMSE={data['global']['rmse']:.4f}, Walls Found={len(data['walls']['rmse'])}")
        else:
            print(f"  Skipping {name} (Data missing or file not found)")

    if results:
        plot_results(results)
    else:
        print("No valid data found to plot.")