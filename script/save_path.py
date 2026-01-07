#!/usr/bin/env python

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import tf
import os
import sys
import threading
import time

class PathToTUM:
    def __init__(self, output_file, topic_name="/path"):
        # Initialize ROS node
        rospy.init_node("path_to_tum", anonymous=True)

        # Ensure output directory exists
        output_dir = os.path.dirname(output_file)
        if output_dir and not os.path.exists(output_dir):
            os.makedirs(output_dir)
            rospy.loginfo("Created directory: %s", output_dir)

        # Open file and write headers (if needed)
        self.file = open(output_file, "w")

        # Subscribe to the specified path topic
        rospy.Subscriber(topic_name, Path, self.path_callback)

        # Initialize the last message time and message count
        self.last_msg_time = rospy.Time.now()
        self.msg_count = 0

        # Start a separate thread to monitor timeouts
        self.timeout_thread = threading.Thread(target=self.monitor_timeout)
        self.timeout_thread.daemon = True
        self.timeout_thread.start()

        rospy.loginfo("Subscribed to %s. Writing data to %s", topic_name, output_file)

    def path_callback(self, msg):
        # # Update the time of the last received message
        # self.last_msg_time = rospy.Time.now()

        # Increment message count
        self.msg_count += 1

        # rospy.loginfo("Received path message with %d poses. Writing to file...", len(msg.poses))

        # Clear the file before writing the new path (optional, uncomment if needed)
        self.file.seek(0)
        self.file.truncate()

        # Iterate through each pose in the path message
        for pose_stamped in msg.poses:
            # Get timestamp
            timestamp = "%.6f" % pose_stamped.header.stamp.to_sec()

            # Extract position information (tx, ty, tz)
            position = pose_stamped.pose.position
            tx, ty, tz = position.x, position.y, position.z

            # Extract quaternion (qx, qy, qz, qw)
            orientation = pose_stamped.pose.orientation
            qx, qy, qz, qw = orientation.x, orientation.y, orientation.z, orientation.w

            # Format as TUM format
            tum_line = f"{timestamp} {tx:.6f} {ty:.6f} {tz:.6f} {qx:.6f} {qy:.6f} {qz:.6f} {qw:.6f}\n"

            # Write to file
            self.file.write(tum_line)

        # Ensure data is written to disk immediately
        self.file.flush()
        # rospy.loginfo("Finished writing %d poses to file.", len(msg.poses))

        self.update_path(msg)

    def update_path(self, new_path):
        # # 判断last_path是否存在且pose数量一致
        # if hasattr(self, 'last_path') and self.last_path is not None and len(self.last_path.poses) == len(new_path.poses):
        #     same = True
        #     for p1, p2 in zip(self.last_path.poses, new_path.poses):
        #         # 注意：PoseStamped对象的位姿在p1.pose.position和p1.pose.orientation
        #         if (p1.pose.position.x != p2.pose.position.x or
        #             p1.pose.position.y != p2.pose.position.y or
        #             p1.pose.position.z != p2.pose.position.z or
        #             p1.pose.orientation.x != p2.pose.orientation.x or
        #             p1.pose.orientation.y != p2.pose.orientation.y or
        #             p1.pose.orientation.z != p2.pose.orientation.z or
        #             p1.pose.orientation.w != p2.pose.orientation.w):
        #             same = False
        #             break
        #     if same:
        #         # 路径完全一样，不更新last_msg_time
        #         return

        # 路径有变化，更新last_path和last_msg_time
        self.last_path = new_path
        self.last_msg_time = rospy.Time.now()

    def monitor_timeout(self):
        # Note: This timeout checks for inactivity on the path topic.
        # If the path is published only once or infrequently, this might shut down the node.
        # Adjust the timeout duration or logic if needed based on expected path publishing behavior.
        timeout_duration = rospy.get_param("~timeout", 30.0) # Default timeout 10 seconds
        while not rospy.is_shutdown():
            # Check if any messages have been processed
            if self.msg_count > 0:
                # Calculate time since last message
                time_since_last_msg = (rospy.Time.now() - self.last_msg_time).to_sec()

                # Check if the timeout has been exceeded
                if time_since_last_msg > timeout_duration:
                    rospy.logwarn("No path messages received for %.1f seconds. Shutting down...", timeout_duration)
                    rospy.signal_shutdown(f"No path messages received for {timeout_duration} seconds")
                    break

            elif (rospy.Time.now() - rospy.get_rostime()).to_sec() > timeout_duration:
                # If no messages received since node start after timeout duration
                rospy.logwarn("No path messages received since node start for %.1f seconds. Shutting down...", timeout_duration)
                rospy.signal_shutdown(f"No path messages received since node start for {timeout_duration} seconds")
                break

            # Sleep for a short interval before checking again
            rospy.sleep(0.1)

    def run(self):
        # Keep the node running
        rospy.spin()

        # Close the file when the node is shut down
        if self.file and not self.file.closed:
            self.file.close()
            rospy.loginfo("Output file closed.")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python save_path.py <output_file> [topic_name]")
        print("  output_file: Path to the output TUM file.")
        print("  topic_name: (Optional) Name of the path topic to subscribe to (default: /path).")
        sys.exit(1)

    output_file = sys.argv[1]
    topic_name = "/path" # Default topic name
    if len(sys.argv) >= 3:
        topic_name = sys.argv[2]

    try:
        path_to_tum = PathToTUM(output_file, topic_name)
        path_to_tum.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS node interrupted.")
    finally:
        # Ensure file is closed even if there's an error during initialization or run
        if 'path_to_tum' in locals() and hasattr(path_to_tum, 'file') and path_to_tum.file and not path_to_tum.file.closed:
             path_to_tum.file.close()
             rospy.loginfo("Output file closed during shutdown.")
