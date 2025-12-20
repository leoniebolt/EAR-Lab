#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path

# CONFIGURATION
# LIO-SAM outputs the full optimized path here
TOPIC_NAME = "/lio_sam/mapping/path"
OUTPUT_FILE = "liosam_global_path.txt"

class GlobalPathSaver(Node):
    def __init__(self):
        super().__init__('global_path_saver')
        self.subscription = self.create_subscription(
            Path, TOPIC_NAME, self.listener_callback, 10)
        self.latest_path = None
        self.get_logger().info(f"Waiting for path on {TOPIC_NAME}...")

    def listener_callback(self, msg):
        # Store the latest (longest) path in memory
        self.latest_path = msg
        count = len(msg.poses)
        # Only print every 100 updates to keep terminal clean
        if count % 100 == 0: 
            self.get_logger().info(f"Current path length: {count} poses")

    def save_to_file(self):
        if self.latest_path is None:
            self.get_logger().warn("No path received! Did LIO-SAM run?")
            return

        print(f"Saving {len(self.latest_path.poses)} poses to {OUTPUT_FILE}...")
        with open(OUTPUT_FILE, "w") as f:
            for pose_stamped in self.latest_path.poses:
                # 1. Timestamp
                sec = pose_stamped.header.stamp.sec
                nsec = pose_stamped.header.stamp.nanosec
                timestamp = f"{sec}.{nsec:09d}"

                # 2. Position & Rotation
                p = pose_stamped.pose.position
                o = pose_stamped.pose.orientation
                
                # 3. Write Line (TUM format)
                line = f"{timestamp} {p.x:.4f} {p.y:.4f} {p.z:.4f} {o.x:.4f} {o.y:.4f} {o.z:.4f} {o.w:.4f}\n"
                f.write(line)
        print("File saved successfully.")

def main(args=None):
    rclpy.init(args=args)
    saver = GlobalPathSaver()
    try:
        rclpy.spin(saver)
    except KeyboardInterrupt:
        # Save when you press Ctrl+C
        saver.save_to_file()
    finally:
        saver.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()