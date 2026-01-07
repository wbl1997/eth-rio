#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Convert sensor_msgs/PointCloud (with channels) to sensor_msgs/PointCloud2 in RIO CFAR format.

RIO expects PointCloud2 fields:
  x(float32), y(float32), z(float32), doppler(float32), snr(int16), noise(int16)

NTU4DRadLM provides /radar_enhanced_pcl as sensor_msgs/PointCloud:
  - msg.points: geometry_msgs/Point32 list (x,y,z)
  - msg.channels: e.g. Doppler, Range, Power, Alpha, Beta

This node maps:
  doppler <- channel (default: Doppler)
  snr     <- channel (default: Power), scaled and cast to int16
  noise   <- constant 0
"""

import argparse
import math
import rospy
from sensor_msgs.msg import PointCloud, PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2


def get_channel_map(channels):
    """Return dict: name -> list[float]."""
    m = {}
    for ch in channels:
        m[ch.name] = ch.values
    return m


def clamp_int16(v: float) -> int:
    if math.isnan(v) or math.isinf(v):
        v = 0.0
    v = int(round(v))
    if v > 32767:
        return 32767
    if v < -32768:
        return -32768
    return v


def build_fields():
    return [
        PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
        PointField(name='doppler', offset=12, datatype=PointField.FLOAT32, count=1),
        PointField(name='snr', offset=16, datatype=PointField.INT16, count=1),
        PointField(name='noise', offset=18, datatype=PointField.INT16, count=1),
    ]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--pc1-in', default='/radar_enhanced_pcl')
    ap.add_argument('--pc2-out', default='/radar/cfar_detections')
    ap.add_argument('--out-frame', default='ntu_radar', help='Force output frame_id (avoid TF frame conflicts).')
    ap.add_argument('--doppler-channel', default='Doppler')
    ap.add_argument('--snr-channel', default='Power')
    ap.add_argument('--snr-scale', type=float, default=1.0)
    ap.add_argument('--snr-bias', type=float, default=0.0)
    ap.add_argument('--node-name', default='pc1_to_cfar_pc2')
    args = ap.parse_args(rospy.myargv()[1:])

    rospy.init_node(args.node_name, anonymous=False)
    pub = rospy.Publisher(args.pc2_out, PointCloud2, queue_size=2)
    fields = build_fields()

    def cb(msg: PointCloud):
        ch = get_channel_map(msg.channels)
        dop = ch.get(args.doppler_channel, None)
        snr = ch.get(args.snr_channel, None)

        n = len(msg.points)
        if dop is None or len(dop) != n:
            rospy.logwarn_throttle(2.0, "Missing/size-mismatch doppler channel '%s' (have=%s). Using 0.",
                                   args.doppler_channel, list(ch.keys()))
            dop = [0.0] * n
        if snr is None or len(snr) != n:
            rospy.logwarn_throttle(2.0, "Missing/size-mismatch snr channel '%s' (have=%s). Using 0.",
                                   args.snr_channel, list(ch.keys()))
            snr = [0.0] * n

        header = msg.header
        header.frame_id = args.out_frame

        # Build point tuples in the exact field order
        pts = []
        for i, p in enumerate(msg.points):
            snr_i = snr[i] * args.snr_scale + args.snr_bias
            pts.append((
                float(p.x), float(p.y), float(p.z),
                float(dop[i]),
                clamp_int16(snr_i),
                0,
            ))

        out = pc2.create_cloud(header, fields, pts)
        out.is_dense = True
        pub.publish(out)

    rospy.Subscriber(args.pc1_in, PointCloud, cb, queue_size=2)
    rospy.loginfo("pc1->cfar_pc2: %s -> %s (frame=%s)", args.pc1_in, args.pc2_out, args.out_frame)
    rospy.spin()


if __name__ == '__main__':
    main()