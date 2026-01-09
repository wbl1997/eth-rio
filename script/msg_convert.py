#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import copy
import math
import os
import sys
import yaml
import numpy as np
import struct

import rospy
from sensor_msgs.msg import PointCloud2, PointField, Imu
import sensor_msgs.point_cloud2 as pc2


# =============================
# PointField datatype packing
# =============================
_PF_TO_STRUCT = {
    PointField.INT8: "b",
    PointField.UINT8: "B",
    PointField.INT16: "h",
    PointField.UINT16: "H",
    PointField.INT32: "i",
    PointField.UINT32: "I",
    PointField.FLOAT32: "f",
    PointField.FLOAT64: "d",
}

DT_STR_TO_PF = {
    "int8": PointField.INT8,
    "uint8": PointField.UINT8,
    "int16": PointField.INT16,
    "uint16": PointField.UINT16,
    "int32": PointField.INT32,
    "uint32": PointField.UINT32,
    "float32": PointField.FLOAT32,
    "float64": PointField.FLOAT64,
}

PF_TO_NP = {
    PointField.INT8: np.int8,
    PointField.UINT8: np.uint8,
    PointField.INT16: np.int16,
    PointField.UINT16: np.uint16,
    PointField.INT32: np.int32,
    PointField.UINT32: np.uint32,
    PointField.FLOAT32: np.float32,
    PointField.FLOAT64: np.float64,
}


def create_cloud_no_len(header, fields, points_iter, is_dense=True):
    """
    Create PointCloud2 from an iterator/generator without requiring len(points).
    Assumes fields are tightly packed (offsets match packed layout).
    (Your build_tgt_fields packs tightly, so this is OK.)
    """
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


def dtype_size(pf_datatype: int) -> int:
    return np.dtype(PF_TO_NP[pf_datatype]).itemsize


def build_tgt_fields(tgt_fields_spec):
    """
    tgt_fields_spec: list of dict: {name, datatype, count}
    offsets tightly packed, in given order.
    """
    fields = []
    offset = 0
    for s in tgt_fields_spec:
        name = s["name"]
        dt = s.get("datatype", "float32")
        if isinstance(dt, str):
            dt = DT_STR_TO_PF[dt.lower()]
        count = int(s.get("count", 1))
        fields.append(PointField(name=name, offset=offset, datatype=int(dt), count=count))
        offset += dtype_size(int(dt)) * count
    return fields


def cast_value_to_pf(v, pf_datatype: int):
    np_t = PF_TO_NP[pf_datatype]
    try:
        return np_t(v).item()
    except Exception:
        return np_t(0).item()


def validate_profile_cfg(profile_name: str, cfg: dict):
    if not isinstance(cfg, dict):
        raise ValueError(f"profile '{profile_name}' config is not a dict")
    if "tgt_fields" not in cfg or not cfg["tgt_fields"]:
        raise ValueError(f"profile '{profile_name}' missing 'tgt_fields'")
    if "map" not in cfg or not isinstance(cfg["map"], dict):
        raise ValueError(f"profile '{profile_name}' missing 'map' dict")


def pc2_remap(msg: PointCloud2, cfg: dict, debug: bool = False) -> PointCloud2:
    out_header = copy.deepcopy(msg.header)
    if cfg.get("out_frame_id"):
        out_header.frame_id = cfg["out_frame_id"]
    # 你这里强行指定 frame_id，我保留你的行为，但建议后续改为配置项
    out_header.frame_id = cfg.get("force_frame_id", out_header.frame_id)
    if not out_header.frame_id:
        out_header.frame_id = msg.header.frame_id

    skip_nans = bool(cfg.get("skip_nans", True))
    rflt = cfg.get("range_filter")
    rmin = float(rflt.get("min", 0.0)) if rflt else None
    rmax = float(rflt.get("max", 1e30)) if rflt else None

    ds_step = int(cfg.get("downsample_step", 1))
    if ds_step < 1:
        ds_step = 1

    tgt_fields_spec = cfg["tgt_fields"]
    tgt_fields = build_tgt_fields(tgt_fields_spec)
    tgt_names = [f.name for f in tgt_fields]
    tgt_info = {f.name: (f.datatype, f.count) for f in tgt_fields}

    mapping = cfg.get("map", {}) or {}

    # ---------- 输入字段真实顺序（关键修复点） ----------
    in_names = [f.name for f in msg.fields]          # msg里真实顺序
    in_index = {n: i for i, n in enumerate(in_names)}  # name -> tuple index
    in_set = set(in_names)

    # 需要从输入里取哪些字段（不依赖顺序）
    need_src = set()
    for tgt in tgt_names:
        rule = mapping.get(tgt, {})
        src = rule.get("from")
        if src:
            need_src.add(src)
    if rflt:
        need_src.update(["x", "y", "z"])

    # 只保留输入里存在的字段
    need_src = [s for s in need_src if s in in_set]
    need_idx = [in_index[s] for s in need_src]

    if debug:
        rospy.loginfo("Target fields: %s", tgt_names)
        rospy.loginfo("Input fields: %s", in_names)
        rospy.loginfo("Need src fields: %s", need_src)

    def apply_rule(rule, src_val):
        if src_val is None:
            if "default" in rule:
                src_val = rule["default"]
            else:
                src_val = 0

        if "scale" in rule:
            src_val = src_val * float(rule["scale"])
        if "bias" in rule:
            src_val = src_val + float(rule["bias"])
        return src_val

    # 仅打印前N个点，避免卡死
    debug_print_n = int(cfg.get("debug_print_n", 3)) if debug else 0
    printed = 0

    def gen_points():
        nonlocal printed

        # 注意：这里 field_names 传 in_names（真实顺序），确保 tuple index 可用
        for i, p in enumerate(pc2.read_points(msg, field_names=in_names, skip_nans=skip_nans)):
            if i % ds_step != 0:
                continue

            # 按 name->idx 取值，构造 d（不会再错配）
            d = {need_src[i]: p[need_idx[i]] for i in range(len(need_src))}
            # print(d)

            if debug and printed < debug_print_n:
                rospy.loginfo("need_src order=%s", need_src)
                rospy.loginfo("d=%s", d)
                printed += 1

            # range filter
            if rflt and all(k in d for k in ("x", "y", "z")):
                x, y, z = d["x"], d["y"], d["z"]
                rr = math.sqrt(x * x + y * y + z * z)
                if rr < rmin or rr > rmax:
                    continue

            out_vals = []
            for tgt in tgt_names:
                rule = mapping.get(tgt, {})
                src_name = rule.get("from")
                src_val = d.get(src_name, None) if src_name else None

                v = apply_rule(rule, src_val)

                pf_dt, count = tgt_info[tgt]
                if count != 1:
                    # 允许 default 给 list/tuple
                    if isinstance(v, (list, tuple)) and len(v) == count:
                        out_vals.extend([cast_value_to_pf(v[i], pf_dt) for i in range(count)])
                    else:
                        out_vals.extend([cast_value_to_pf(0, pf_dt)] * count)
                else:
                    out_vals.append(cast_value_to_pf(v, pf_dt))

            yield tuple(out_vals)

    out_msg = create_cloud_no_len(out_header, tgt_fields, gen_points(), is_dense=msg.is_dense)
    out_msg.is_dense = msg.is_dense
    return out_msg


# =============================
# Node
# =============================
class Pc2ImuRemapNodeArgs:
    def __init__(self, pc2_in, pc2_out, imu_in, imu_out, profile_cfg, debug=False):
        self.profile_cfg = profile_cfg
        self.debug = debug

        self.pc2_pub = rospy.Publisher(pc2_out, PointCloud2, queue_size=5)
        self.imu_pub = rospy.Publisher(imu_out, Imu, queue_size=50)

        rospy.Subscriber(pc2_in, PointCloud2, self.cb_pc2, queue_size=5)
        rospy.Subscriber(imu_in, Imu, self.cb_imu, queue_size=200)

        rospy.loginfo("pc2 remap active (profile loaded). pc2:%s->%s imu:%s->%s",
                      pc2_in, pc2_out, imu_in, imu_out)

    def cb_pc2(self, msg: PointCloud2):
        try:
            out = pc2_remap(msg, self.profile_cfg, debug=self.debug)
            self.pc2_pub.publish(out)
        except Exception as e:
            rospy.logwarn_throttle(2.0, "pc2 remap failed: %s", str(e))

    def cb_imu(self, msg: Imu):
        # IMU：纯转发（你这里强行改 frame_id，我保留但建议用配置）
        msg.header.frame_id = self.profile_cfg.get("force_imu_frame_id", msg.header.frame_id)
        self.imu_pub.publish(msg)


def load_config(path: str) -> dict:
    if not os.path.exists(path):
        raise FileNotFoundError(f"config not found: {path}")
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict) or "profiles" not in data:
        raise ValueError("config must be a dict with top-level key 'profiles'")
    if not isinstance(data["profiles"], dict):
        raise ValueError("'profiles' must be a dict")
    return data


def parse_args(argv):
    ap = argparse.ArgumentParser(description="ROS1 PC2 remap by YAML profile, IMU passthrough.")
    ap.add_argument("--config", required=True, help="Path to YAML config containing 'profiles'.")
    ap.add_argument("--profile", required=True, help="Profile name under config['profiles'].")
    ap.add_argument("--pc2-in", default="/points_raw", help="Input PointCloud2 topic.")
    ap.add_argument("--pc2-out", default="/points_out", help="Output PointCloud2 topic.")
    ap.add_argument("--imu-in", default="/imu_raw", help="Input IMU topic.")
    ap.add_argument("--imu-out", default="/imu_out", help="Output IMU topic.")
    ap.add_argument("--downsample-step", type=int, default=1, help="Downsample step (keep 1 point every N).")
    ap.add_argument("--node-name", default="pc2_imu_field_remap_args", help="ROS node name.")
    ap.add_argument("--debug", action="store_true", help="Print a few points and mappings for debug.")
    return ap.parse_args(argv)


def main():
    argv = rospy.myargv(argv=sys.argv)[1:]
    args = parse_args(argv)

    cfg_all = load_config(args.config)
    profiles = cfg_all["profiles"]

    if args.profile not in profiles:
        raise KeyError(f"profile '{args.profile}' not found. available: {list(profiles.keys())}")

    profile_cfg = copy.deepcopy(profiles[args.profile])
    validate_profile_cfg(args.profile, profile_cfg)

    # 命令行参数覆盖配置文件
    if args.downsample_step > 1:
        profile_cfg["downsample_step"] = args.downsample_step

    # 你原来强制 frame_id 的行为搬到配置里（可选）
    # profile_cfg["force_frame_id"] = "awr1843aop"
    # profile_cfg["force_imu_frame_id"] = "bmi088"

    rospy.init_node(args.node_name, anonymous=False)
    Pc2ImuRemapNodeArgs(args.pc2_in, args.pc2_out, args.imu_in, args.imu_out, profile_cfg, debug=args.debug)
    rospy.spin()


if __name__ == "__main__":
    main()
