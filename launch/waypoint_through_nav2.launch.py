from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg = get_package_share_directory('waypoint_editor')

    declare_waypoint_file = DeclareLaunchArgument(
        'waypoint_file',
        default_value='',
        description='Full path to waypoint CSV file')

    declare_frame_id = DeclareLaunchArgument(
        'frame_id',
        default_value='map',
        description='Frame ID for waypoints')

    waypoint_file = LaunchConfiguration('waypoint_file')
    frame_id = LaunchConfiguration('frame_id')

    waypoint_through_nav2_node = Node(
        package='waypoint_editor',
        executable='waypoint_through_nav2',
        name='waypoint_through_nav2',
        output='screen',
        parameters=[{
            'waypoint_file': waypoint_file,
            'frame_id': frame_id
        }]
    )

    ld = LaunchDescription()
    ld.add_action(declare_waypoint_file)
    ld.add_action(declare_frame_id)
    ld.add_action(waypoint_through_nav2_node)
    return ld
