#!/usr/bin/env python
import rospy
import struct
import sys
import argparse
from sensor_msgs.msg import PointCloud, PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import std_msgs.msg

import math

class PointCloudConverter:
    def __init__(self):
        # Parse arguments
        parser = argparse.ArgumentParser()
        parser.add_argument('--input_topic', default='/radar_pcl')
        parser.add_argument('--output_topic', default='/radar_pcl2')
        parser.add_argument('--power_threshold', type=float, default=0.0)
        parser.add_argument('--downsample_interval', type=int, default=1)
        parser.add_argument('--target_frame', default='base_link')
        
        args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])

        rospy.init_node('pc_to_pc2_converter', anonymous=True)

        # Parameters
        self.input_topic = args.input_topic
        self.output_topic = args.output_topic
        self.power_threshold = args.power_threshold
        self.downsample_interval = args.downsample_interval
        self.target_frame = args.target_frame
        
        # Subscribers and Publishers
        self.sub = rospy.Subscriber(self.input_topic, PointCloud, self.callback)
        self.pub = rospy.Publisher(self.output_topic, PointCloud2, queue_size=10)

        rospy.loginfo(f"Converter started: {self.input_topic} -> {self.output_topic}")

    def callback(self, msg):
        # Prepare header
        header = std_msgs.msg.Header()
        header.stamp = msg.header.stamp
        # Use the target frame if specified, otherwise keep original
        header.frame_id = self.target_frame if self.target_frame else msg.header.frame_id

        # Find channel indices
        # Default to 0 for Doppler/Velocity and 2 for Intensity/Power based on provided C++ code
        idx_velocity = 0
        idx_intensity = 2
        
        # Try to find by name if possible
        for i, channel in enumerate(msg.channels):
            if "doppler" in channel.name.lower() or "velocity" in channel.name.lower():
                idx_velocity = i
            if "intensity" in channel.name.lower() or "power" in channel.name.lower():
                idx_intensity = i

        points_data = []
        
        # Check bounds
        if idx_velocity >= len(msg.channels) or idx_intensity >= len(msg.channels):
            rospy.logwarn_throttle(5, "Required channels not found in PointCloud message")
            return

        # Iterate through points
        for i, point in enumerate(msg.points):
            # Downsample
            if i % self.downsample_interval != 0:
                continue

            # Check for NaN or Inf
            if math.isnan(point.x) or math.isnan(point.y) or math.isnan(point.z):
                continue
            if math.isinf(point.x) or math.isinf(point.y) or math.isinf(point.z):
                continue

            # Get intensity/power
            intensity = msg.channels[idx_intensity].values[i]
            
            # Filter by power threshold
            if intensity <= self.power_threshold:
                continue
                
            # Get velocity/doppler
            velocity = msg.channels[idx_velocity].values[i]
            
            # Add to list: [x, y, z, intensity, velocity]
            # Note: Coordinate transformation (Radar_to_livox) is NOT applied here.
            # If coordinate transformation is needed, it should be handled via TF 
            # or by modifying the x, y, z values here.
            points_data.append([point.x, point.y, point.z, intensity, velocity])

        # Define fields for PointCloud2
        # struct Point { float x, y, z; float intensity; float velocity; };
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name='velocity', offset=16, datatype=PointField.FLOAT32, count=1),
        ]

        # Create PointCloud2 message
        pc2_msg = pc2.create_cloud(header, fields, points_data)
        
        self.pub.publish(pc2_msg)

if __name__ == '__main__':
    try:
        converter = PointCloudConverter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
