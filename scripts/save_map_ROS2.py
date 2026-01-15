#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import sys

TOPIC_NAME = "/topic_path_to_cloud_points"
OUTPUT_FILE = "name_of_file.pcd"
SKIP_FACTOR = 10

class PCDAccumulator(Node):

    def __init__(self):
        super().__init__('accumulate_pcd_saver')

        self.all_points = []
        self.total_frames_seen = 0
        self.saved_frames_count = 0

        self.get_logger().info(f"Subscribing to: {TOPIC_NAME}")
        self.get_logger().info(f"Saving every {SKIP_FACTOR}th scan.")
        self.get_logger().info("Press CTRL+C when bag finishes.")

        self.create_subscription(
            PointCloud2,
            TOPIC_NAME,
            self.callback,
            10
        )

    def callback(self, msg):
        self.total_frames_seen += 1

        if self.total_frames_seen % SKIP_FACTOR != 0:
            return

        gen = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        new_points = list(gen)
        self.all_points.extend(new_points)

        self.saved_frames_count += 1

        sys.stdout.write(
            f"\rScanned: {self.total_frames_seen} | "
            f"Saved: {self.saved_frames_count} | "
            f"Total Points: {len(self.all_points)}"
        )
        sys.stdout.flush()

    def save_pcd(self):
        num_points = len(self.all_points)

        if num_points == 0:
            self.get_logger().warn("No points received. Nothing to save.")
            return

        self.get_logger().info(f"\nSaving {num_points} points to {OUTPUT_FILE}")

        header = f"""# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z
SIZE 4 4 4
TYPE F F F
COUNT 1 1 1
WIDTH {num_points}
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS {num_points}
DATA ascii
"""

        with open(OUTPUT_FILE, 'w') as f:
            f.write(header)
            for p in self.all_points:
                f.write(f"{p[0]:.4f} {p[1]:.4f} {p[2]:.4f}\n")

        self.get_logger().info("PCD file written successfully.")

def main():
    rclpy.init()
    node = PCDAccumulator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_pcd()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

