module_delay=8

gnome-terminal -x bash -c "source /opt/ros/melodic/setup.bash;roslaunch /opt/ros/melodic/share/mavros/launch/px4.launch; exec bash -i"
sleep ${module_delay}

gnome-terminal -x bash -c "source ~/waypoint_ws/devel/setup.bash;roslaunch /home/nvidia/waypoint_ws/src/off_mission/launch/off_mission.launch; exec bash -i"
sleep ${module_delay}

gnome-terminal -x bash -c "source ~/waypoint_ws/devel/setup.bash;roslaunch /home/nvidia/waypoint_ws/src/navigator/launch/navigator.launch; exec bash -i"
sleep ${module_delay}
