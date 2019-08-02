#!/bin/bash

today=`date +'%Y_%m_%d'`
time=`date +'%H_%M_%S'`
dir=/mnt/SamsungKURI/master_study_bag/$today

if [ -e $dir ]; then
	echo "file found"
else
	mkdir -p $dir
fi

echo "record_start"/image_color
# rosbag record /camera_info /closest_obstacle /detected_obstacles /managed_obstacles /image_color /int_pointcloud /next_waypoint_mark /swipe_erase_signal /twist_raw /ypspur_ros/cmd_vel /ypspur_ros/odom /points_raw /swipe_visualizer_node/update /swipe_visualizer_node/update_full /joy /tf /projection_matrix -O $dir/$time.bag
rosbag record　/shifted_info /closest_obstacle /detected_obstacles /managed_obstacles /next_waypoint_mark /swipe_erase_signal /twist_raw /ypspur_ros/cmd_vel /ypspur_ros/odom /points_raw /joy
