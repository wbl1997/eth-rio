#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
import tf
import os
import sys
import threading

class OdomToTUM:
    def __init__(self, output_file):
        # Initialize ROS node
        rospy.init_node("odom_to_tum", anonymous=True)

        # Ensure output directory exists
        output_dir = os.path.dirname(output_file)
        if output_dir and not os.path.exists(output_dir):
            os.makedirs(output_dir)
            rospy.loginfo("Created directory: %s", output_dir)

        # Open file and write headers (if needed)
        self.file = open(output_file, "w")

        # Subscribe to /odom topic
        rospy.Subscriber("/Odometry", Odometry, self.odom_callback)

        # Initialize the last message time and message count
        self.last_msg_time = rospy.Time.now()
        self.msg_count = 0

        # Start a separate thread to monitor timeouts
        self.timeout_thread = threading.Thread(target=self.monitor_timeout)
        self.timeout_thread.daemon = True
        self.timeout_thread.start()

        rospy.loginfo("Subscribed to /Odometry. Writing data to %s", output_file)

    def odom_callback(self, msg):
        # Update the time of the last received message
        self.last_msg_time = rospy.Time.now()

        # Increment message count
        self.msg_count += 1

        # Get timestamp
        timestamp = "%.6f" % msg.header.stamp.to_sec()

        # Extract position information (tx, ty, tz)
        position = msg.pose.pose.position
        tx, ty, tz = position.x, position.y, position.z

        # Extract quaternion (qx, qy, qz, qw)
        orientation = msg.pose.pose.orientation
        qx, qy, qz, qw = orientation.x, orientation.y, orientation.z, orientation.w

        # Format as TUM format
        tum_line = f"{timestamp} {tx:.6f} {ty:.6f} {tz:.6f} {qx:.6f} {qy:.6f} {qz:.6f} {qw:.6f}\n"

        # Write to file
        self.file.write(tum_line)

        # Optional: Print to console for debugging
        rospy.loginfo("Written: %s", tum_line.strip())

    def monitor_timeout(self):
        while not rospy.is_shutdown():
            # Check if any messages have been processed
            if self.msg_count > 0:
                # Calculate time since last message
                time_since_last_msg = rospy.Time.now() - self.last_msg_time

                # Check if the timeout has been exceeded
                if time_since_last_msg.to_sec() > 10.0:
                    rospy.logwarn("No odometry messages received for 10 seconds. Shutting down...")
                    rospy.signal_shutdown("No odometry messages received for 10 seconds")
                    break

            # Sleep for a short interval before checking again
            rospy.sleep(0.1)

    def run(self):
        # Keep the node running
        rospy.spin()

        # Close the file
        self.file.close()

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python save_odom.py <output_file>")
        sys.exit(1)

    output_file = sys.argv[1]

    try:
        odom_to_tum = OdomToTUM(output_file)
        odom_to_tum.run()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS node interrupted.")
