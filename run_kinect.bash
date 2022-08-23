source dev_ws/devel/setup.bash
roslaunch azure_kinect_ros_driver driver.launch fps:=15 body_tracking_smoothing_factor:=0 rgb_point_cloud:=false body_tracking_enabled:=true
