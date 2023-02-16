# CUSTOM_GAIT
Author: Katie Hughes

This is a ROS2 package that will generate desired gaits for the Unitree Go1 robot.

Before you can use this package, use the `unitree.repos` file in order to also clone the `unitree_ros2` package into the same workspace.

## Executables
* `custom_gait`: This takes the framework defined in Unitree's `ros2_position_example` and translates it into a more traditional ROS2 C++ node. A trajectory for each foot is generated and published using Bezier curve. It also subscribes to the `low_state` message from the Go1 to read from the sensors on the feet, althought there is no further implementation beyond this. In order for this node to physically move the dog, `udp_low` from the `unitree_legged_real` package must also be running.
  * Parameter `rate`: Publish rate in Hz. Default = 200 Hz.
  * Parameter `seconds_per_swing`: How many seconds it takes for one leg to complete a full step. Default = 2.0 sec.

## Launchfiles
* `ros2 launch custom_gait custom_gait.launch.py`: This launches the `custom_gait` node, the `udp_low` node from the `unitree_legged_real` package (to send generated gaits to the real robot), and the `visualize.launch.py` launchfile from the `go1_description` package (to visualize the movements in rviz). Even if you are not physically connected to the robot, you will still see the robot move.


Here is a video of a gait I generated using an 11-th order Bezier curve (12 control points), plus a sinusoidal section for the "strike" portion of the swing (which connects the two endpoints of the Bezier curve).


[Screencast from 02-10-2023 02:12:07 PM.webm](https://user-images.githubusercontent.com/53623710/218196692-e11d45ed-0b68-4e45-9935-aba4aa1d3c14.webm)
