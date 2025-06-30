from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import AppendEnvironmentVariable
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, TextSubstitution, PathJoinSubstitution

import os

def generate_launch_description():
    # launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')
    launchers_package = get_package_share_directory('cachecons_launchers')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')

    world = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'worlds',
        'empty_world.world'
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r -s -v2 ', world], 'on_exit_shutdown': 'true'}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-g -v2 '}.items()
    )

    set_env_vars_resources = AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.path.join(
                get_package_share_directory('turtlebot3_gazebo'),
                'models'))

    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(set_env_vars_resources)

    r1_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launchers_package, 'launch', 'burger_launch.py')
        ),
        launch_arguments={'namespace': 'r1', 'y': '0.0'}.items()
    )

    r2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launchers_package, 'launch', 'burger_launch.py')
        ),
        launch_arguments={'namespace': 'r2', 'y': '1.0'}.items()
    )

    ld.add_action(r1_cmd)
    ld.add_action(r2_cmd)
    
    return ld

