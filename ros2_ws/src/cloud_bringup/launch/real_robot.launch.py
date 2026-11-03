"""
Project Cloud: Native ROS 2 Launch Configuration for Physical Robot
Deploys on Ubuntu 22.04 LTS with Jetson Orin companion compute.
"""

def generate_launch_description():
    """ROS 2 launch description for field deployment on Unitree Go2."""
    from launch import LaunchDescription
    from launch_ros.actions import Node

    return LaunchDescription([
        Node(
            package='cloud_hardware_bridge',
            executable='unitree_sport_client',
            name='unitree_sport_client',
            output='screen'
        ),
        Node(
            package='cloud_perception',
            executable='aruco_tracker_node',
            name='aruco_tracker_node',
            output='screen'
        ),
        Node(
            package='cloud_perception',
            executable='floor_spill_detector',
            name='floor_spill_detector',
            output='screen'
        ),
        Node(
            package='cloud_navigation',
            executable='optical_follower_node',
            name='optical_follower_node',
            output='screen'
        ),
        Node(
            package='cloud_navigation',
            executable='presenter_pivot_node',
            name='presenter_pivot_node',
            output='screen'
        ),
        Node(
            package='cloud_inventory',
            executable='inventory_service_node',
            name='inventory_service_node',
            output='screen'
        ),
        Node(
            package='cloud_cognition',
            executable='speech_pipeline_node',
            name='speech_pipeline_node',
            output='screen'
        ),
        Node(
            package='cloud_fsm',
            executable='cloud_supervisory_fsm',
            name='cloud_supervisory_fsm',
            output='screen'
        ),
        Node(
            package='cloud_webhooks',
            executable='discord_dispatcher',
            name='discord_dispatcher',
            output='screen'
        ),
    ])
