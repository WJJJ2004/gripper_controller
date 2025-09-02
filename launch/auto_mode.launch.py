from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_path = get_package_share_directory('gripper_controller')
    urdf_path = os.path.join(pkg_path, 'robot_model', 'humanoid_mani.urdf')
    srdf_path = os.path.join(pkg_path, 'robot_model', 'humanoid_mani.srdf')
    param_path = os.path.join(pkg_path, 'config', 'ros_param.yaml')

    gripper_main_node = Node(
        package='gripper_controller',
        executable='gripper_main_node',
        name='gripper_main_node',
        output='screen',
        parameters=[param_path],
        arguments=[urdf_path, srdf_path, "auto"]
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': open(urdf_path, 'r').read()}]
    )

    return LaunchDescription([
        gripper_main_node,
        robot_state_publisher_node
    ])
