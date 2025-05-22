from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Package directory
    pkg_share = FindPackageShare('leptrino_force_torque_sensor')
    
    # Declare launch arguments
    com_port_arg = DeclareLaunchArgument(
        'com_port', 
        default_value='/dev/ttyACM0', 
        description='Serial port to connect to'
    )
    
    auto_recalibration_arg = DeclareLaunchArgument(
        'auto_recalibration_enabled',
        default_value='false',
        description='Enable automatic recalibration'
    )
    
    recalibration_interval_arg = DeclareLaunchArgument(
        'auto_recalibration_interval_minutes',
        default_value='30.0',
        description='Interval between auto-recalibrations in minutes'
    )
    
    calibration_samples_arg = DeclareLaunchArgument(
        'calibration_samples',
        default_value='100',
        description='Number of samples for calibration'
    )

    # Parameters file path
    params_file = PathJoinSubstitution([
        pkg_share,
        'config',
        'params.yaml'
    ])

    # Create the leptrino node
    leptrino_node = Node(
        package='leptrino_force_torque_sensor',
        executable='leptrino_force_torque_sensor',
        name='leptrino_sensor',
        parameters=[
            params_file,
            {
                'com_port': LaunchConfiguration('com_port'),
                'auto_recalibration_enabled': LaunchConfiguration('auto_recalibration_enabled'),
                'auto_recalibration_interval_minutes': LaunchConfiguration('auto_recalibration_interval_minutes'),
                'calibration_samples': LaunchConfiguration('calibration_samples'),
            }
        ],
        output='screen',
        respawn=True,
        respawn_delay=2.0
    )

    return LaunchDescription([
        com_port_arg,
        auto_recalibration_arg,
        recalibration_interval_arg,
        calibration_samples_arg,
        leptrino_node
    ])
