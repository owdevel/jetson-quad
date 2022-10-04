#!/bin/bash

rosbag record -e /camera/imu "/cpu_monitor/(.*)" /orbslam_trajectory /orbslam_time /tf /tf_static
