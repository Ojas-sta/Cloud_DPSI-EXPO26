"""
Project Cloud: Gazebo Simulator ROS 2 Launch Manifest
Launches Gazebo Fortress/Harmonic with the DPSI EXPO Pavilion world and Go2 Edu robot.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = os.path.dirname(os.path.abspath(__file__))
    world_path = os.path.join(pkg_share, "cloud_gazebo.world")
    urdf_path = os.path.join(pkg_share, "unitree_go2.urdf")

    with open(urdf_path, "r") as f:
        robot_desc = f.read()

    return LaunchDescription([
        # Robot State Publisher
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_desc, "use_sim_time": True}]
        ),
        # Spawn Entity in Gazebo
        Node(
            package="ros_gz_sim",
            executable="create",
            arguments=["-name", "unitree_go2", "-string", robot_desc, "-z", "0.35"],
            output="screen"
        ),
        # ROS 2 <-> Gazebo Bridge
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=[
                "/cloud/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
                "/cloud/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry",
                "/cloud/camera/rgb/image_raw@sensor_msgs/msg/Image@gz.msgs.Image",
            ],
            output="screen"
        )
    ])
