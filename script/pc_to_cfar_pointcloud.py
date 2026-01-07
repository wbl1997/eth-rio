#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import math
import os
import sys
import yaml
import struct

import rospy
from sensor_msgs.msg import PointCloud, PointCloud2, PointField
from geometry_msgs.msg import Point32

# -----------------------------
# PointField packing helpers
# -----------------------------
_PF_TO_STRUCT = {
    PointField.INT16: "h",
    PointField.FLOAT32: "f",
}

def create_cloud_no_len(header, fields, points_iter, is_dense=True):
    fmt = "<"
    for f in fields:
        if f.datatype not in _PF_TO_STRUCT:
            raise ValueError(f"Unsupported datatype: {f.datatype} for field {f.name}")
        fmt += _PF_TO_STRUCT[f.datatype] * int(f.count)

    packer = struct.Struct(fmt)
    point_step = packer.size

    buf = bytearray()
    width = 0
    for pt in points_iter:
        buf += packer.pack(*pt)
        width += 1

    cloud = PointCloud2()
    cloud.header = header
    cloud.height = 1
    cloud.width = width
    cloud.fields = fields
    cloud.is_bigendian = False
    cloud.point_step = point_step
    cloud.row_step = point_step * width
    cloud.is_dense = bool(is_dense)
    cloud.data = bytes(buf)
    return cloud

def clamp_int16(v: float) -> int:
    v = int(round(v))
    if v > 32767:
        return 32767
    if v < -32768:
        return -32768
    return v

def load_cfg(path: str) -> dict:
    if not os.path.exists(path):
        raise FileNotFoundError(f"config not found: {path}")
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict) or "profiles" not in data:
        raise ValueError("config must be a dict with top-level key 'profiles'")
    return data

def parse_args(argv):
    ap = argparse.ArgumentParser("Convert sensor_msgs/PointCloud to RIO CFAR PointCloud2")
    ap.add_argument("--config", required=True)
    ap.add_argument("--profile", required=True)
    ap.add_argument("--pc-in", default="/radar_enhanced_pcl")
    ap.add_argument("--pc2-out", default="/radar/cfar_detections")
    ap.add_argument("--node-name", default="pc_to_cfar_pointcloud")
    ap.add_argument("--debug", action="store_true")
    return ap.parse_args(argv)

class PcToCfar:
    def __init__(self, pc_in: str, pc2_out: str, profile: dict, debug: bool):
        self.profile = profile
        self.debug = debug

        self.pub = rospy.Publisher(pc2_out, PointCloud2, queue_size=5)
        rospy.Subscriber(pc_in, PointCloud, self.cb, queue_size=2)

        rospy.loginfo("pc->cfar active. %s -> %s (profile loaded)", pc_in, pc2_out)

        # target fields for RIO
        # x y z doppler : float32
        # snr noise     : int16
        self.fields = [
            PointField(name="x", offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name="doppler", offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name="snr", offset=16, datatype=PointField.INT16,   count=1),
            PointField(name="noise", offset=18, datatype=PointField.INT16, count=1),
        ]

    def cb(self, msg: PointCloud):
        ch_map = {c.name: c.values for c in msg.channels}

        doppler_name = self.profile.get("doppler_channel", "Doppler")
        snr_name = self.profile.get("snr_channel", "Power")

        skip_nans = bool(self.profile.get("skip_nans", True))
        snr_scale = float(self.profile.get("snr_scale", 1.0))
        snr_bias  = float(self.profile.get("snr_bias", 0.0))
        noise_default = float(self.profile.get("noise_default", 0.0))

        doppler_vals = ch_map.get(doppler_name, None)
        snr_vals = ch_map.get(snr_name, None)

        if doppler_vals is None:
            rospy.logwarn_throttle(2.0, "channel '%s' not found in PointCloud.channels", doppler_name)
            return
        if snr_vals is None:
            rospy.logwarn_throttle(2.0, "channel '%s' not found in PointCloud.channels", snr_name)
            return

        n = len(msg.points)
        if len(doppler_vals) != n or len(snr_vals) != n:
            rospy.logwarn_throttle(
                2.0,
                "channel length mismatch: points=%d doppler=%d snr=%d",
                n, len(doppler_vals), len(snr_vals)
            )
            return

        out_header = msg.header
        force_frame = self.profile.get("force_frame_id")
        if force_frame:
            out_header.frame_id = force_frame

        # generator to avoid huge temporary list
        def gen():
            kept = 0
            for i, p in enumerate(msg.points):
                if skip_nans and (math.isnan(p.x) or math.isnan(p.y) or math.isnan(p.z)):
                    continue
                dop = float(doppler_vals[i])
                snr_f = float(snr_vals[i]) * snr_scale + snr_bias
                snr_i = clamp_int16(snr_f)
                noise_i = clamp_int16(noise_default)
                kept += 1
                yield (float(p.x), float(p.y), float(p.z), dop, snr_i, noise_i)

        out = create_cloud_no_len(out_header, self.fields, gen(), is_dense=True)
        self.pub.publish(out)

        if self.debug:
            rospy.loginfo_throttle(1.0, "published cfar cloud, frame_id=%s", out.header.frame_id)

def main():
    argv = rospy.myargv(argv=sys.argv)[1:]
    args = parse_args(argv)

    cfg_all = load_cfg(args.config)
    profiles = cfg_all["profiles"]
    if args.profile not in profiles:
        raise KeyError(f"profile '{args.profile}' not found. available: {list(profiles.keys())}")
    profile = profiles[args.profile]
    if not isinstance(profile, dict):
        raise ValueError("profile config must be a dict")

    rospy.init_node(args.node_name, anonymous=False)
    PcToCfar(args.pc_in, args.pc2_out, profile, debug=args.debug)
    rospy.spin()

if __name__ == "__main__":
    main()
