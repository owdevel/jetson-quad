#!/bin/bash

rosbag record -e /camera/imu "/cpu_monitor/(.*)" /orbslam_trajectory /orbslam_time /tf /tf_static
#rosbag record -e /camera/imu "/cpu_monitor/(.*)" /tf /tf_static /camera/color/image_raw /camera/aligned_depth_to_color/image_raw
