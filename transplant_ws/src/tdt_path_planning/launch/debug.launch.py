from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    configuration = os.path.join(
        get_package_share_directory('tdt_path_planning'), 'cfg', 'planner.yaml')
    return LaunchDescription([
        Node(package='tdt_path_planning', executable='snapshot_planner',
             name='snapshot_planner', output='screen', parameters=[configuration])
    ])
