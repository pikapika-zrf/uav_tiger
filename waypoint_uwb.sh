module_delay=8

gnome-terminal -x bash -c "source ~/uwb_ws/devel/setup.bash;roslaunch px4_uwb swarm0.launch; exec bash -i"
sleep ${module_delay}

gnome-terminal -x bash -c "source /opt/ros/melodic/setup.bash;roslaunch /opt/ros/melodic/share/mavros/launch/px4.launch; exec bash -i"
sleep ${module_delay}

gnome-terminal -x bash -c "source ~/UWB_ws/devel/setup.bash;roslaunch /home/nvidia/UWB_ws/src/off_mission/launch/off_mission.launch; exec bash -i"
sleep ${module_delay}

gnome-terminal -x bash -c "source ~/UWB_ws/devel/setup.bash;roslaunch /home/nvidia/UWB_ws/src/navigator/launch/navigator.launch; exec bash -i"
sleep ${module_delay}
