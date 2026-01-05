# ping_monitor
A lightweight ROS 2 node that monitors network connectivity using system `ping`

# Install
```bash
cd path/to/your_ws/src
git clone 
cd path/to/your_ws
colcon build --packages-select ping_monitor
source install/setup.bash
```
# Usage
```bash
ros2 run ping_monitor ping_monitor_node --ros-args -p target_host:="8.8.8.8" -p ping_interval:=0.5
```

# Parameter
- `target_host` (string, default: 8.8.8.8): Target IP address or hostname.

- `ping_interval` (float, default: 1.0): Ping interval in seconds.