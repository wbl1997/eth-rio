wspath="/home/wbl/code/biye_data_process/code/eth_rio_ws/"

gnome-terminal -x bash -c "roscore" & sleep 3

gnome-terminal -x bash -c "
source $wspath/devel/setup.bash
roslaunch rio rio.launch
" & sleep 2

gnome-terminal -x bash -c "
rosbag play /home/wbl/下载/01_urban_night_H_raw_no_img.bag  -s 0 -r 3
" &

