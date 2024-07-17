from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare the launch argument
    declare_usb_device_arg = DeclareLaunchArgument(
        'usb_device',
        default_value='/dev/ttyUSB0',
        description='USB device for micro-ROS agent'
    )

    # Use LaunchConfiguration to get the value of the launch argument
    usb_device = LaunchConfiguration('usb_device')

    # Create the command to run the Docker container with the substitution
    docker_command = [
        'docker', 'stop', '$(docker ps --quiet --filter ancestor=microros/micro-ros-agent:humble)',
        '&&'
        'docker', 'run', '--rm', 
        '-v', '/dev:/dev', 
        '-v', '/dev/shm:/dev/shm', 
        '--privileged', '--net=host', 
        'microros/micro-ros-agent:$ROS_DISTRO', 
        'serial', '--dev', usb_device
    ]

    # Define the ExecuteProcess action
    execute_docker_process = ExecuteProcess(
        cmd=docker_command,
        shell=True
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add the DeclareLaunchArgument and ExecuteProcess action to the launch description
    ld.add_action(declare_usb_device_arg)
    ld.add_action(execute_docker_process)

    return ld
