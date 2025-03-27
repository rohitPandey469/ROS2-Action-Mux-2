# ROS2 Universal Subscriber/Publisher System

## Installation

1. Clone the repository:
```bash
git clone https://github.com/rohitPandey469/ROS2-Action-Mux-2.git
cd ROS2-Action-Mux-2/
rosdep install --from-paths src --ignore-src -y --rosdistro humble
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

2. Terminal 1 (Universal subscriber)
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run universal_subscriber universal_subscriber
```

3. Terminal2 (Test Publisher)
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run universal_subscriber test_publishers
```
