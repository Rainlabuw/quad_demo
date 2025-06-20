cd ~/Downloads/KentTempTest/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build
ros2 run crazyflie_bridge crazyfliebridge
