#! /bin/bash

mkdir /home/oarbot_silver/eric/human2robot/bags/$(date +%m%d_%H%M)

rosbag record -o /home/oarbot_silver/eric/human2robot/bags/$(date +%m%d_%H%M)/human --split --size=2048 \
	/tf \
	/tf_static \
	/rgb/camera_info \
	/rgb/image_raw \
	/depth/camera_info \
	/depth/image_raw \
	/body_tracking_data 
