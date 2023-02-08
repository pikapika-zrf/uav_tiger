module_delay=8

gnome-terminal -x bash -c "source /opt/ros/melodic/setup.bash;roslaunch mavros px4.launch; exec bash -i"
sleep ${module_delay}

gnome-terminal -x bash -c "source ~/realsense_ws/devel/setup.bash;roslaunch /home/nvidia/realsense_ws/src/off_mission/launch/off_mission.launch; exec bash -i"
sleep ${module_delay}

gnome-terminal -x bash -c "source ~/realsense_ws/devel/setup.bash;roslaunch /home/nvidia/realsense_ws/src/navigator/launch/navigator.launch; exec bash -i"
sleep ${module_delay}

gnome-terminal -x bash -c "source ~/realsense_ws/devel/setup.bash;roslaunch /home/nvidia/realsense_ws/src/realsense-ros/realsense2_camera/launch/rs_camera_vins.launch; exec bash -i"
sleep ${module_delay}

gnome-terminal -x bash -c "source ~/realsense_ws/devel/setup.bash;roslaunch /home/nvidia/realsense_ws/src/VINS-Fusion/vins_estimator/launch/vins_rviz.launch; exec bash -i"
sleep ${module_delay}

gnome-terminal -x bash -c "source ~/realsense_ws/devel/setup.bash;roslaunch /home/nvidia/realsense_ws/src/vins_px4/launch/vins_uav.launch; exec bash -i"
sleep ${module_delay}



