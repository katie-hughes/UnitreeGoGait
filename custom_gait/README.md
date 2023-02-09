# CUSTOM_GAIT
Author: Katie Hughes

This is a ROS2 package that will generate desired gaits for the Unitree Go1 robot.

Before you can use this package, use the `unitree.repos` file in order to also clone the `unitree_ros2` package into the same workspace.

## Executables
* `custom_gait`: This takes the framework defined in Unitree's `ros2_position_example` and translates it into a more traditional ROS2 C++ node. It also subscribes to the `low_state` message from the Go1 to read from the sensors on the feet. In order for this node to physically move the dog, `udp_low` from the `unitree_legged_real` package must also be running.

## Launchfiles
* `ros2 launch custom_gait custom_gait.launch.py`: This launches the `custom_gait` node, the `udp_low` node from the `unitree_legged_real` package (to send generated gaits to the real robot), and the `visualize.launch.py` launchfile from the `go1_description` package (to visualize the movements in rviz).