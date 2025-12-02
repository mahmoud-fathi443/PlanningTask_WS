from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os



def generate_launch_description():
    rviz_config_dir = get_package_share_directory('planning') + '/rviz'
    rviz_config_file = os.path.join(rviz_config_dir, 'planning_config.rviz') # to load rviz config file

    return LaunchDescription([
        Node(
            package="planning",
            executable="map_creation"
        ),
        Node(
            package="planning",
            executable="global_planner"
        ),
        Node(
            package="planning",
            executable="rviz_visualization"
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=['-d', rviz_config_file],
        ),
    ])