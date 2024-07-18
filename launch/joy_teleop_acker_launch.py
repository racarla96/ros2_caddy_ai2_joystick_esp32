import os

from ament_index_python.packages import get_package_share_directory

import launch
import launch_ros.actions
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    joy_config = launch.substitutions.LaunchConfiguration('joy_config')
    config_filepath = launch.substitutions.LaunchConfiguration('config_filepath')

    joy_launch_file_dir = os.path.join(get_package_share_directory('joystick_bridge'), 'launch')

    usb_device_dir = LaunchConfiguration('usb_device', default='/dev/ttyUSB0')

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument('joy_vel', default_value='cmd_vel'), 
        launch.actions.DeclareLaunchArgument('joy_config', default_value='joy_teleop'),       
        launch.actions.DeclareLaunchArgument('config_filepath', default_value=[
            launch.substitutions.TextSubstitution(text=os.path.join(
                get_package_share_directory('joystick_bridge'), 'config', '')),
            joy_config, launch.substitutions.TextSubstitution(text='.yaml')]),

        DeclareLaunchArgument(
            'usb_device',
            default_value=usb_device_dir,
            description='Full path to map file to load'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([joy_launch_file_dir, '/joy_launch.py']),
            launch_arguments={
                'usb_device': usb_device_dir}.items(),
        ),


        launch_ros.actions.Node(
            package='teleop_acker_joy', executable='teleop_node',
            name='teleop_acker_joy_node',
            parameters=[config_filepath],
            remappings={('/cmd_vel', launch.substitutions.LaunchConfiguration('joy_vel'))},
            ),
    ])