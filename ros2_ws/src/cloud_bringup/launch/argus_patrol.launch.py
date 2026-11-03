"""
Project Cloud: ARGUS Makerspace Workhorse Autonomous Patrol Launch
Configures the quadruped for continuous laboratory surveillance and inventory dispatch.
"""

def generate_launch_description():
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
            executable='floor_spill_detector',
            name='floor_spill_detector',
            output='screen'
        ),
        Node(
            package='cloud_inventory',
            executable='inventory_service_node',
            name='inventory_service_node',
            output='screen'
        ),
        Node(
            package='cloud_webhooks',
            executable='discord_dispatcher',
            name='discord_dispatcher',
            output='screen'
        ),
        Node(
            package='cloud_fsm',
            executable='cloud_supervisory_fsm',
            name='cloud_supervisory_fsm',
            parameters=[{'default_persona': 'ARGUS', 'starting_state': 'STATE_ARGUS_PATROL'}],
            output='screen'
        ),
    ])
