wspath="/home/wbl/code/biye_data_process/code/eth_rio_ws/"

gnome-terminal -x bash -c "roscore" & sleep 3

gnome-terminal -x bash -c "
source $wspath/devel/setup.bash
roslaunch rio rio_coloradar.launch
" & sleep 2

gnome-terminal -x bash -c "
source activate radar_env
python $wspath/src/eth-rio/script/msg_convert.py \
    --config $wspath/src/eth-rio/script/msg_convert_cfg.yaml \
    --profile coloradar \
    --pc2-in /mmWaveDataHdl/RScan \
    --pc2-out /radar/cfar_detections \
    --imu-in /gx5/imu/data \
    --imu-out /imu/data
" & sleep 2

gnome-terminal -x bash -c "
rosrun topic_tools relay /imu/data /imu/data_raw
" & sleep 2

gnome-terminal -x bash -c "
rosbag play /media/wbl/KESU/data/coloradar/rosbags/edgar_classroom_run0.bag  -s 0 -r 1 
" &


