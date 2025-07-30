from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg = get_package_share_directory('waypoint_editor')
    rviz_config = os.path.join(pkg, 'rviz', 'rviz_waypoint_editor.rviz')

    declare_map_yaml = DeclareLaunchArgument(
        'map_yaml', default_value=os.path.join(pkg, 'data', 'sample_map.yaml'),
        description='Full path to map yaml file')
    map_yaml_file = LaunchConfiguration('map_yaml')

    map_server = LifecycleNode(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        namespace='',
        output='screen',
        parameters=[{'yaml_filename': map_yaml_file}]
        )
    
    lifecycle_mgr = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        namespace='',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': ['map_server']
        }])

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config]
    )

    ld = LaunchDescription()
    ld.add_action(declare_map_yaml)
    ld.add_action(map_server)
    ld.add_action(lifecycle_mgr)
    ld.add_action(rviz2)
    return ld