#!/usr/bin/env bash
set -e

# NTU4DRadLM (cp_2022-02-26.bag) -> ETH RIO pipeline
# - Radar input:  sensor_msgs/PointCloud  (/radar_enhanced_pcl)
# - IMU input:    sensor_msgs/Imu         (/vectornav/imu)
# - Output topics expected by RIO:
#     /imu/data_raw (Imu), /imu/data (Imu), /radar/cfar_detections (PointCloud2)

wspath="/home/lucas/jjl/eth_rio_wbl"
BAG="/media/lucas/KINGSTON/wbl/data/NTU4DRadLM/cp/cp_2022-02-26.bag"

# Frame names used by our pipeline (avoid conflicts with frames in /tf from the bag)
IMU_FRAME="imu_frame"         # from /vectornav/imu header.frame_id
RADAR_FRAME="ntu_radar"       # our synthetic radar frame for RIO

# --- 0) roscore
(gnome-terminal -- bash -lc "roscore") &
sleep 2

# --- 1) use_sim_time must be true BEFORE starting RIO
(gnome-terminal -- bash -lc "rosparam set use_sim_time true") &
sleep 1

# --- 2) start RIO with our dataset launch
(gnome-terminal -- bash -lc "
source ${wspath}/devel/setup.bash
roslaunch rio rio_ntu4dradlm.launch imu_frame_id:=${IMU_FRAME} radar_frame_id:=${RADAR_FRAME} visualize:=false
") &
sleep 2

# --- 3) relay IMU to the topics RIO expects
# NOTE: topic_tools relay keeps message content (including frame_id) unchanged.
(gnome-terminal -- bash -lc "
rosrun topic_tools relay /vectornav/imu /imu/data_raw
") &
sleep 1

(gnome-terminal -- bash -lc "
rosrun topic_tools relay /vectornav/imu /imu/data
") &
sleep 1

# --- 4) convert radar PointCloud -> PointCloud2 in RIO format
(gnome-terminal -- bash -lc "
source ${wspath}/devel/setup.bash
python3 ${wspath}/src/eth-rio/script/pc1_to_cfar_pc2.py \
  --pc1-in /radar_pcl \
  --pc2-out /radar/cfar_detections \
  --out-frame ${RADAR_FRAME} \
  --doppler-channel Doppler \
  --snr-channel Power \
  --snr-scale 1.0
") &
sleep 2

# --- 5) play bag (publish /clock)
# (gnome-terminal -- bash -lc "
# rosbag play ${BAG} --clock -r 1 -s 0
# ") &
# --- 5) play bag (publish /clock)
BAG="/media/lucas/KINGSTON/wbl/data/NTU4DRadLM/cp/cp_2022-02-26.bag"

(gnome-terminal -- bash -lc "
source /opt/ros/noetic/setup.bash
rosparam set use_sim_time true

rosbag play \"$BAG\" --clock -r 0.5 -s 1.0 --topics \
  /vectornav/imu \
  /radar_pcl
exec bash
") &



