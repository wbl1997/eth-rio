#!/usr/bin/env bash
set -e

wspath="/home/lucas/jjl/eth_rio_wbl"
bag="/media/lucas/KINGSTON/wbl/data/msc/RURAL_A0.bag"

# 1) roscore
gnome-terminal -- bash -lc "roscore; exec bash" & sleep 3

# 使用 bag 的 /clock（推荐）
rosparam set /use_sim_time true || true

# 2) 启动 RIO（MSC launch）
gnome-terminal -- bash -lc "
source /opt/ros/noetic/setup.bash
source $wspath/devel/setup.bash
roslaunch rio rio_msc.launch visualize:=true
exec bash
" & sleep 2

# 3) 点云字段转换：/oculii_radar/point_cloud -> /radar/cfar_detections
#    注意：MSC 的 IMU 只有 /imu/data，我们不让 msg_convert 去“重复发布”IMU（避免双发布）
gnome-terminal -- bash -lc "
# conda 兼容 anaconda/miniconda
if [ -f \"\$HOME/anaconda3/etc/profile.d/conda.sh\" ]; then
  source \"\$HOME/anaconda3/etc/profile.d/conda.sh\"
elif [ -f \"\$HOME/miniconda3/etc/profile.d/conda.sh\" ]; then
  source \"\$HOME/miniconda3/etc/profile.d/conda.sh\"
fi
conda activate radar_env

source /opt/ros/noetic/setup.bash
source $wspath/devel/setup.bash

python $wspath/src/eth-rio/script/msg_convert.py \
  --config $wspath/src/eth-rio/script/msg_convert_cfg.yaml \
  --profile msc \
  --pc2-in /oculii_radar/point_cloud \
  --pc2-out /radar/cfar_detections \
  --imu-in /__dummy_imu_in \
  --imu-out /__dummy_imu_out

exec bash
" & sleep 2

# 4) IMU raw：把 /imu/data 复制成 /imu/data_raw（RIO需要 data_raw + data）
gnome-terminal -- bash -lc "
source /opt/ros/noetic/setup.bash
rosrun topic_tools relay /imu/data /imu/data_raw
exec bash
" & sleep 2

# 5) 播包（带 /clock）
gnome-terminal -- bash -lc "
rosbag play \"$bag\" -s 0 -r 1
exec bash
" &
