#!/usr/bin/env bash
set -e

wspath="/home/lucas/jjl/eth_rio_wbl"
bag="/media/lucas/KINGSTON/wbl/data/snail_radar/20240113/data1_aligned.bag"

# 1) roscore
gnome-terminal -- bash -lc "roscore" & sleep 3

# 2) RIO (snail launch)
gnome-terminal -- bash -lc "
source $wspath/devel/setup.bash
roslaunch rio rio_snail_radar.launch visualize:=true
" & sleep 2

# 3) msg_convert: /ars548 -> /radar/cfar_detections, /imu_raw -> /imu/data_raw
gnome-terminal -- bash -lc "
# conda 兼容 anaconda/miniconda
if [ -f \"\$HOME/anaconda3/etc/profile.d/conda.sh\" ]; then
  source \"\$HOME/anaconda3/etc/profile.d/conda.sh\"
elif [ -f \"\$HOME/miniconda3/etc/profile.d/conda.sh\" ]; then
  source \"\$HOME/miniconda3/etc/profile.d/conda.sh\"
fi
conda activate radar_env

python $wspath/src/eth-rio/script/msg_convert.py \
  --config $wspath/src/eth-rio/script/msg_convert_cfg.yaml \
  --profile snail_radar \
  --pc2-in /ars548 \
  --pc2-out /radar/cfar_detections \
  --imu-in /imu_raw \
  --imu-out /imu/data_raw
" & sleep 2

# 4) play bag (带 /clock)
gnome-terminal -- bash -lc "
rosbag play --clock \"$bag\" -s 0 -r 1
" &
