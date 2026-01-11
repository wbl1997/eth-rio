wspath="$HOME/jjl/code/ETH_WBL"

gnome-terminal -x bash -c "roscore" & sleep 3

gnome-terminal -x bash -c "
source $wspath/devel/setup.bash
export LD_LIBRARY_PATH=$wspath/devel/lib:$LD_LIBRARY_PATH
roslaunch rio rio_ntu4dradlm.launch
" & sleep 2

gnome-terminal -x bash -c "
source activate radar_env
python $wspath/src/eth-rio/script/pc_to_pc2.py \
    --downsample_interval 5 --input_topic /radar_enhanced_pcl
" & sleep 3


gnome-terminal -x bash -c "
source activate radar_env
python $wspath/src/eth-rio/script/msg_convert.py \
  --config $wspath/src/eth-rio/script/msg_convert_cfg.yaml \
  --profile ntu \
  --pc2-in /radar_pcl2 \
  --pc2-out /radar/cfar_detections  \
  --imu-in /livox/imu \
  --imu-out /imu/data/
  --downsample-step 3
" & sleep 2

gnome-terminal -x bash -c "
rosrun topic_tools relay /imu/data /imu/data_raw
" & sleep 2

gnome-terminal -x bash -c "
rosbag play /media/lucas/KINGSTON/wbl/data/NTU4DRadLM/cp/cp_2022-02-26.bag  -s 0 -r 1 
" &

