cd ~/<ros_workspace>/
source /opt/ros/galactic/setup.bash 
colcon build --packages-select tour_robot_pkg
source install/local_setup.bash
ros2 run tour_robot_pkg tour_robot_node