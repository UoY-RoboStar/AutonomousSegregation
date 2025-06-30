import os

from ament_index_python.packages import get_package_share_directory

from irobot_create_common_bringup.namespace import GetNamespacedName
from irobot_create_common_bringup.offset import OffsetParser, RotationalOffsetX, RotationalOffsetY

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node, PushRosNamespace, SetRemap
from launch.substitutions import PythonExpression,PathJoinSubstitution

ARGUMENTS = [
    DeclareLaunchArgument('namespace', default_value='',
                          description='Robot namespace'),
]

for pose_element in ['x', 'y', 'z', 'yaw']:
    ARGUMENTS.append(DeclareLaunchArgument(pose_element, default_value='0.0',
                     description=f'{pose_element} component of the robot pose.'))
    

def generate_start_bridge_and_spawner(context):
    TURTLEBOT3_MODEL = 'burger'
    model_folder = 'turtlebot3_' + TURTLEBOT3_MODEL
    ns = LaunchConfiguration('namespace').perform(context)
    print("NAMESPACE: ", ns)
    bridge_params = os.path.join(
        # get_package_share_directory('turtlebot3_gazebo'),
        get_package_share_directory('cachecons_launchers'),
        'params',
        ns,
        model_folder+'_bridge.yaml'
    )

    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ],
        output='screen',
    )

    urdf_path = os.path.join(
        get_package_share_directory('cachecons_launchers'),
        'model',
        ns,
        model_folder,
        'model.sdf'
    )

    start_gazebo_ros_spawner_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', LaunchConfiguration('namespace'),
            '-file', urdf_path,
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', '0.01'
        ],
        output='screen',
    )

    return [start_gazebo_ros_bridge_cmd, start_gazebo_ros_spawner_cmd]

    
def generate_launch_description():
    # Directories
    pkg_turtlebot3_gazebo = get_package_share_directory(
        'turtlebot3_gazebo')
    
    start_bridge_and_spawner=OpaqueFunction(function=generate_start_bridge_and_spawner)
    namespace = LaunchConfiguration('namespace')

    

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # # cmd_vel bridge
    # cmd_vel_bridge = Node(
    #     package='ros_gz_bridge',
    #     executable='parameter_bridge',
    #     name='cmd_vel_bridge',
    #     output='screen',
    #     parameters=[{
    #         'use_sim_time': use_sim_time
    #     }],
    #     arguments=[
    #         [namespace,
    #          '/cmd_vel' + '@geometry_msgs/msg/TwistStamped' + '[ignition.msgs.Twist'],
    #         ['/model/', namespace, '/cmd_vel' +
    #          '@geometry_msgs/msg/TwistStamped' +
    #          ']ignition.msgs.Twist']
    #     ],
    #     remappings=[
    #         ([namespace, '/cmd_vel'], 'cmd_vel'),
    #         (['/model/', namespace, '/cmd_vel'],
    #          'diffdrive_controller/cmd_vel')
    #     ])

    # # Pose bridge
    # pose_bridge = Node(package='ros_gz_bridge', executable='parameter_bridge',
    #                    name='pose_bridge',
    #                    output='screen',
    #                    parameters=[{
    #                         'use_sim_time': use_sim_time
    #                    }],
    #                    arguments=[
    #                        ['/model/', namespace, '/pose' +
    #                         '@tf2_msgs/msg/TFMessage' +
    #                         '[ignition.msgs.Pose_V'],
    #                    ],
    #                    remappings=[
    #                        (['/model/', namespace, '/pose'],
    #                         '_internal/sim_ground_truth_pose'),
    #                    ])

    # # odom to base_link transform bridge
    # odom_base_tf_bridge = Node(package='ros_gz_bridge', executable='parameter_bridge',
    #                            name='odom_base_tf_bridge',
    #                            output='screen',
    #                            parameters=[{
    #                                'use_sim_time': use_sim_time
    #                            }],
    #                            arguments=[
    #                                ['/model/', namespace, '/tf' +
    #                                 '@tf2_msgs/msg/TFMessage' +
    #                                 '[ignition.msgs.Pose_V']
    #                            ],
    #                            remappings=[
    #                                (['/model/', namespace, '/tf'], 'tf')
    #                            ])




    start_gazebo_ros_image_bridge_cmd = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/camera/image_raw'],
        output='screen',
    )

    sim_time_arg = DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
    )
    
    urdf_file_name = 'turtlebot3_' + 'burger' + '.urdf'
    frame_prefix = LaunchConfiguration('frame_prefix', default='')

    print('urdf_file_name : {}'.format(urdf_file_name))

    urdf_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'urdf',
        urdf_file_name)

    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_desc,
                'frame_prefix': PythonExpression(["'", frame_prefix, "/'"])
            }],
            # remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        )
    
    
    burger_with_namespace = GroupAction(
        actions=[
            PushRosNamespace(namespace),
            SetRemap('/tf', 'tf'),
            SetRemap('/tf_static', 'tf_static'),
            # cmd_vel_bridge,
            # pose_bridge,
            # odom_base_tf_bridge,
            state_publisher,
            start_bridge_and_spawner,
            # start_gazebo_ros_bridge_cmd,
            start_gazebo_ros_image_bridge_cmd,
            sim_time_arg,
        ]
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(burger_with_namespace)
    # ld.add_action(start_gazebo_ros_image_bridge_cmd) if TURTLEBOT3_MODEL != 'burger' else None
    
    
    return ld
    