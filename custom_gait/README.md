# CUSTOM_GAIT
Author: Katie Hughes

This is a ROS2 package that will generate desired gaits for the Unitree Go1 robot.

Before you can use this package, use the `unitree.repos` file in order to also clone the `unitree_ros2` package into the same workspace.

More information on this project can be found here: https://katie-hughes.github.io/unitree/

## Executables
* `custom_gait`: Provides a low level implemenation to stand up, lie down, and walk forwards, backwards, left, and right. A trajectory for each foot is generated and published using Bezier curve. In order for this node to physically move the dog, `udp_low` from the `unitree_legged_real` package must also be running.
### Services
* `ros2 service call /forwards std_srvs/srv/Empty`: Begin walking forwards
* `ros2 service call /backwards std_srvs/srv/Empty`: Begin walking backwards
* `ros2 service call /left std_srvs/srv/Empty`: Begin walking left
* `ros2 service call /right std_srvs/srv/Empty`: Begin walking right
* `ros2 service call /lie_down std_srvs/srv/Empty`: Lie down
* `ros2 service call /stand_up std_srvs/srv/Empty`: Stand up
### Important Node Parameters
  * `rate`: Publish rate in Hz. Default = 200 Hz.
  * `seconds_per_swing`: How many seconds it takes for one leg to complete a full step. Default = 0.25 sec.
  * `stiffness`: Kp for the motors. Default: 90.0
  * `damping`: Kd for the motors. Default: 5.0
  * `stroke_length_x, dx1, dx2`: Bezier curve parameters for taking steps forward and backwards.
  * `stroke_length_z, dz1, dz2`: Bezier curve parameters for taking steps forward and backwards.
  * `step_height, dy`: Bezier curve parameters that affect the height of steps for both forward and sideways steps.
  * `gait_type`: 0 moves only a single leg. 1 is a "tripod" walk that will always keep 3 legs on the ground at any given time. 2 is default and "trot" gait. The backwards and left services will not work well if the gait is not trot. 
  * `trot_offset`: 0 for a continuous trot, any higher integer for a delay between steps. The backwards and left services will not work well for an offset greater than 0.
  * `step_limit`: True if you want to stop walking after `nsteps` steps (useful for real world testing), false if you want a continuous walk (useful for simulation).
  * `stand_percentage`: This determines how high off the ground the robot is during standing as a percentage of its maximum height. 0.75 is default. 

## Launchfiles
* `ros2 launch custom_gait custom_gait.launch.py`: This launches the `custom_gait` node, the `udp_low` node from the `unitree_legged_real` package (to send generated gaits to the real robot), and the `visualize.launch.py` launchfile from the `go1_description` package (to visualize the movements in rviz). Even if you are not physically connected to the robot, you will still see the robot move.


Here is a video of a gait I generated using an 11-th order Bezier curve (12 control points), plus a sinusoidal section for the "strike" portion of the swing (which connects the two endpoints of the Bezier curve). This showcases the forward, backwards, left, right, lie down, and stand up procedures. The translucent model corresponds to the commanded joint positions and the opaque model corresponds to the real robot state. 


https://user-images.githubusercontent.com/53623710/225983409-d9467a48-2375-406c-9097-98405262f74f.mp4
