wspath="~/jjl/code/ETH_WBL"

gnome-terminal -x bash -c "roscore" & sleep 3

gnome-terminal -x bash -c "
source $wspath/devel/setup.bash
roslaunch rio rio_msc.launch visualize:=true
" & sleep 2

gnome-terminal -x bash -c "
source activate radar_env
python $wspath/src/eth-rio/script/msg_convert.py \
    --config $wspath/src/eth-rio/script/msg_convert_cfg.yaml \
    --profile msc \
    --pc2-in /oculii_radar/point_cloud \
    --pc2-out /radar/cfar_detections \
    --imu-in /__dummy_imu_in \
    --imu-out /__dummy_imu_out \
    --downsample-step 10
" & sleep 2

gnome-terminal -x bash -c "
rosrun topic_tools relay /imu/data /imu/data_raw
" & sleep 2

gnome-terminal -x bash -c "
rosbag play /media/lucas/KINGSTON/wbl/data/msc/RURAL_A0.bag -s 0 -r 1
" &


