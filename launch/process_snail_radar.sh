wspath="~/jjl/code/ETH_WBL"

gnome-terminal -x bash -c "roscore" & sleep 3

gnome-terminal -x bash -c "
source $wspath/devel/setup.bash
roslaunch rio rio_snail_radar.launch
" & sleep 2

gnome-terminal -x bash -c "
source activate radar_env
python $wspath/src/eth-rio/script/msg_convert.py \
    --config $wspath/src/eth-rio/script/msg_convert_cfg.yaml \
    --profile snail_radar \
    --pc2-in /ars548 \
    --pc2-out /radar/cfar_detections \
    --imu-in /imu_raw0 \
    --imu-out /imu/data_raw0 \
    --downsample-step 3
" & sleep 2

gnome-terminal -x bash -c "
rosrun topic_tools relay /imu/data /imu/data_raw
" & sleep 2

gnome-terminal -x bash -c "
rosbag play /media/lucas/KINGSTON/wbl/data/snail_radar/20240113/data1_aligned.bag  -s 0 -r 3 
" &


