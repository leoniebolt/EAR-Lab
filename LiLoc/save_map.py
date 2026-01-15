#!/usr/bin/env python
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
import sys

# --- CONFIGURATION ---
TOPIC_NAME = "/liloc/mapping/cloud_registered"
OUTPUT_FILE = "map/accumulated_map_sparse.pcd"

# Save only every Nth scan. 
# 10 means we save 1 scan, ignore 9, save 1... (approx 1 scan per second)
SKIP_FACTOR = 10 
# ---------------------

all_points = []
total_frames_seen = 0
saved_frames_count = 0

def save_pcd_and_exit():
    """Writes the accumulated points to a PCD file."""
    global all_points, saved_frames_count
    
    num_points = len(all_points)
    
    if num_points == 0:
        rospy.logwarn("No points received yet. Nothing to save.")
        return

    rospy.loginfo("\n------------------------------------------------")
    rospy.loginfo("STOPPING: Preparing to save data...")
    rospy.loginfo("Processed %d total frames.", total_frames_seen)
    rospy.loginfo("Accumulated %d frames (Skipping every %d).", saved_frames_count, SKIP_FACTOR)
    rospy.loginfo("Total points to write: %d", num_points)
    rospy.loginfo("Writing to %s (this may take a minute)...", OUTPUT_FILE)

    try:
        # Write PCD Header
        header = """# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z
SIZE 4 4 4
TYPE F F F
COUNT 1 1 1
WIDTH {}
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS {}
DATA ascii
""".format(num_points, num_points)

        with open(OUTPUT_FILE, 'w') as f:
            f.write(header)
            for p in all_points:
                # p is a tuple (x, y, z)
                f.write("{:.4f} {:.4f} {:.4f}\n".format(p[0], p[1], p[2]))

        rospy.loginfo("SUCCESS: Map saved to %s", OUTPUT_FILE)
    except Exception as e:
        rospy.logerr("Failed to write file: %s", str(e))

def callback(msg):
    global all_points, total_frames_seen, saved_frames_count
    
    total_frames_seen += 1

    # MODULO CHECK: Only process if the remainder is 0
    if total_frames_seen % SKIP_FACTOR != 0:
        return

    # If we pass the check, save this frame
    gen = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
    new_points = list(gen)
    all_points.extend(new_points)
    
    saved_frames_count += 1
    
    # Update status on the same line to reduce clutter
    sys.stdout.write("\rScanned: {} | Saved: {} | Total Points: {}".format(
        total_frames_seen, saved_frames_count, len(all_points)))
    sys.stdout.flush()

def main():
    rospy.init_node('accumulate_pcd_saver')
    
    rospy.loginfo("Subscribing to: %s", TOPIC_NAME)
    rospy.loginfo("Saving every %dth scan.", SKIP_FACTOR)
    rospy.loginfo("Press CTRL+C when bag finishes to save the file.")
    
    rospy.Subscriber(TOPIC_NAME, PointCloud2, callback)
    
    # Register the save function to run when the node shuts down
    rospy.on_shutdown(save_pcd_and_exit)
    
    rospy.spin()

if __name__ == '__main__':
    main()
