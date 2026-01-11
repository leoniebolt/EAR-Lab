#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Path
import sys

# CONFIGURATION
TOPIC_NAME = "/lio_sam/mapping/path"
OUTPUT_FILE = "liosam_raw_trajectory.txt"

class PathRecorder(Node):
    def __init__(self):
        super().__init__('path_recorder')
        
        # Use BEST_EFFORT to ensure we catch the topic even if QoS settings vary
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.subscription = self.create_subscription(
            Path, 
            TOPIC_NAME, 
            self.listener_callback, 
            qos_profile
        )
        
        self.latest_path = None
        self.get_logger().info(f"Waiting for Path data on {TOPIC_NAME}...")
        self.get_logger().info("The script will capture the FULL trajectory. Press Ctrl+C when the run is finished to save.")

    def listener_callback(self, msg):
        # Always update to the latest full path message
        self.latest_path = msg
        count = len(msg.poses)
        self.get_logger().info(f"Tracking path: {count} poses received...", throttle_duration_sec=5.0)

    def save_to_file(self):
        if self.latest_path is None:
            self.get_logger().warn("No path received! File not saved.")
            return

        pose_count = len(self.latest_path.poses)
        self.get_logger().info(f"Saving {pose_count} RAW poses to {OUTPUT_FILE}...")
        
        with open(OUTPUT_FILE, "w") as f:
            # Iterate strictly over the poses provided by the SLAM node
            for pose_stamped in self.latest_path.poses:
                
                # Get timestamp from the specific pose, not the global header
                # This ensures the time corresponds exactly to when that keyframe was created
                t = pose_stamped.header.stamp.sec + pose_stamped.header.stamp.nanosec * 1e-9
                
                p = pose_stamped.pose.position
                q = pose_stamped.pose.orientation
                
                # TUM Format: timestamp x y z qx qy qz qw
                # Using .9f for precision to ensure no data loss
                line = f"{t:.9f} {p.x:.9f} {p.y:.9f} {p.z:.9f} {q.x:.9f} {q.y:.9f} {q.z:.9f} {q.w:.9f}\n"
                f.write(line)

        self.get_logger().info(f"Successfully saved {pose_count} poses. No interpolation applied.")

    def destroy_node(self):
        # Save automatically on shutdown
        self.save_to_file()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    recorder = PathRecorder()
    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        pass # Allow clean exit to trigger save
    finally:
        recorder.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()