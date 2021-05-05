import os
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    ld = LaunchDescription()

    # Set env var to print messages to stdout immediately
    arg = SetEnvironmentVariable('RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')
    ld.add_action(arg)
    
    driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            launch_file_path=PathJoinSubstitution([
                FindPackageShare('bluespace_ai_xsens_mti_driver'), 'launch', 'xsens_mti_node.launch.py'
                ]),
            )
        )
    ld.add_action(driver_launch)

    # Rviz2 node
    rviz_config_path = os.path.join(get_package_share_directory('bluespace_ai_xsens_mti_driver'), 'rviz', 'display.rviz')
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='xsens_rviz2',
        output='screen',
        arguments=[["-d"],[rviz_config_path]],
    )
    ld.add_action(rviz2_node)

    # Robot State Publisher node
    urdf_file_path = os.path.join(get_package_share_directory('bluespace_ai_xsens_mti_driver'), 'urdf', 'MTi_6xx.urdf')
    state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='xsens_state_publisher',
        output='screen',
        arguments=[urdf_file_path],
    )
    ld.add_action(state_publisher_node)

    return ld
