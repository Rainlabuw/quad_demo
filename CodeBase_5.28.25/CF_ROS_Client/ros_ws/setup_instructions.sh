cd ~/Downloads/CF_ROS_Client/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build
ros2 run crazyflie_client crazyflie_client
