
"""
Launches low level go controls for the go1 with visualization in rviz.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, Shutdown, SetLaunchConfiguration, IncludeLaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch.conditions import LaunchConfigurationEquals
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
     return LaunchDescription([

          IncludeLaunchDescription(
               PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare('go1_description'),
                                                           'launch',
                                                           'visualize.launch.py'])),
                    launch_arguments=[
                         ('use_jsp', 'none'),
                         ('publish_static_world_tf', 'true'),
                         ('fixed_frame', 'world')
                    ]),

          Node(package='unitree_legged_real',
               executable='udp_low',
               output='screen'),

          Node(package='custom_gait',
               executable='custom_gait',
               output='screen')
    ])