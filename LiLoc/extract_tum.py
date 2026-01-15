#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Path
import os

# Path where you want to save the results
OUTPUT_FILE = "liloc_estimate.txt"

class GlobalPathExtractor(object):
    def __init__(self, topic_name):
        self.topic_name = topic_name
        self.latest_path = None

        # Subscribe to LIO-SAM global path
        self.sub = rospy.Subscriber(self.topic_name, Path, self.callback, queue_size=10)

        rospy.loginfo(f"Waiting for Path data on {self.topic_name}...")
        rospy.loginfo("Use Ctrl+C to stop and save the file.")

        # Helpful: list currently published Path topics
        try:
            topics = rospy.get_published_topics()
            path_topics = [t for t, ty in topics if ty == '/liloc/mapping/path']
            if path_topics:
                rospy.loginfo(f"Detected Path topics: {', '.join(path_topics)}")
            else:
                rospy.logwarn("No nav_msgs/Path topics currently published.")
        except Exception as e:
            rospy.logwarn(f"Unable to query published topics: {e}")

    def callback(self, msg):
        self.latest_path = msg
        count = len(msg.poses)
        if count % 100 == 0:
            rospy.loginfo(f"Current path length: {count} poses")

    def save_to_file(self):
        if self.latest_path is None or not self.latest_path.poses:
            rospy.logwarn("No path received! File not saved.")
            return

        # Overwrite any existing file with the final full path
        try:
            with open(OUTPUT_FILE, "w") as f:
                for pose_stamped in self.latest_path.poses:
                    sec = pose_stamped.header.stamp.secs if hasattr(pose_stamped.header.stamp, 'secs') else pose_stamped.header.stamp.sec
                    nsec = pose_stamped.header.stamp.nsecs if hasattr(pose_stamped.header.stamp, 'nsecs') else pose_stamped.header.stamp.nsec
                    timestamp = float(f"{sec}.{nsec:09d}")

                    p = pose_stamped.pose.position
                    o = pose_stamped.pose.orientation

                    tum_line = f"{timestamp:.9f} {p.x:.4f} {p.y:.4f} {p.z:.4f} {o.x:.4f} {o.y:.4f} {o.z:.4f} {o.w:.4f}\n"
                    f.write(tum_line)
            rospy.loginfo(f"Saved {len(self.latest_path.poses)} poses to {OUTPUT_FILE}")
        except Exception as e:
            rospy.logerr(f"Failed to save file: {e}")

if __name__ == '__main__':
    # Delete old file if it exists to start fresh
    if os.path.exists(OUTPUT_FILE):
        os.remove(OUTPUT_FILE)

    rospy.init_node('tum_data_extractor')

    # Allow overriding via ROS param: ~topic
    topic_name = rospy.get_param('~topic', '/liloc/mapping/path')

    extractor = GlobalPathExtractor(topic_name)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    finally:
        extractor.save_to_file()
