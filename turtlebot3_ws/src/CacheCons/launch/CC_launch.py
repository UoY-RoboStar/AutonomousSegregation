"""Launch the CacheCons simulation on Gazebo Harmonic."""
import importlib.util
import math
import os
from pathlib import Path
import shutil
import tempfile

import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess, GroupAction,
    LogInfo, OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable)
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from CacheCons.gazebo_world import build_world, bridge_config


MACHINES = ('random_walk', 'obstacle_avoidance', 'move_manager',
            'cache_cons_s', 'cache_point_assignment', 'target_watch')


def _robot_namespaces(context):
    count = int(context.launch_configurations['num_robots'])
    if not 1 <= count <= 3:
        raise ValueError('num_robots must be 1, 2, or 3')
    return ['T3A', 'T3B', 'T3C'][:count]


def _boolean(config, name):
    if config[name].lower() not in ('true', 'false'):
        raise ValueError(name + ' must be true or false')
    return config[name].lower() == 'true'


def _robot_specs(config, namespaces, package):
    poses = yaml.safe_load((package / 'config/initial_poses.yaml').read_text())
    if config['calibrate_camera'] != 'none':
        # One robot: the onboard target is placed relative to it, the overhead target is it.
        poses, namespaces = poses['diagnostic'], namespaces[:1]
    return [(namespace, entity, pose) for namespace, (entity, pose) in
            zip(namespaces, poses[len(namespaces)].items())]


def _start(context):
    cfg = context.launch_configurations
    cluster_distance = float(cfg['cluster_distance'])
    if not math.isfinite(cluster_distance) or cluster_distance <= 0.0:
        raise ValueError('cluster_distance must be finite and positive')
    namespaces = _robot_namespaces(context)
    package = Path(get_package_share_directory('CacheCons'))
    try:
        get_package_share_directory('ros_gz_sim')
        get_package_share_directory('ros_gz_bridge')
        if importlib.util.find_spec('gz.transport13') is None:
            raise ImportError('gz.transport13')
        if importlib.util.find_spec('gz.msgs10') is None:
            raise ImportError('gz.msgs10')
    except (LookupError, ImportError) as exc:
        raise RuntimeError('Gazebo Harmonic dependencies are missing. See README.md: '
            'ros_gz_sim, ros_gz_bridge, python3-gz-transport13 and python3-gz-msgs10 '
            'are required.') from exc
    calibrate = cfg['calibrate_camera']
    if calibrate not in ('none', 'onboard', 'overhead'):
        raise ValueError('calibrate_camera must be none, onboard or overhead')
    calibration_output = cfg['calibration_output']
    if calibrate != 'none':
        if not calibration_output:
            raise ValueError('calibrate_camera needs calibration_output:=<csv path>')
        calibration_output = os.path.abspath(calibration_output)
    elif calibration_output:
        raise ValueError('calibration_output is only used with calibrate_camera')
    controller = cfg['controller']
    if controller == 'single':
        machines = ('controller',)
    elif controller == 'nodes':
        machines = MACHINES
    else:
        machines = tuple(name.strip() for name in controller.split(','))
        unknown = [name for name in machines if name not in MACHINES]
        if unknown:
            raise ValueError('controller must be single, nodes or machine names from '
                             + ', '.join(MACHINES) + '; got ' + ', '.join(unknown))
    overhead_rate = float(cfg['overhead_camera_rate'])
    if overhead_rate <= 0:
        raise ValueError('overhead_camera_rate must be positive')
    # Calibration captures a table instead of running controllers, pucks or obstacles.
    simulating = calibrate == 'none'
    overhead_camera = _boolean(cfg, 'overhead_camera') or calibrate == 'overhead'
    specs = _robot_specs(cfg, namespaces, package)
    runtime = Path(tempfile.mkdtemp(prefix='cachecons-harmonic-'))
    world_path, bridge_path = runtime / 'world.sdf', runtime / 'bridge.yaml'
    world_path.write_text(build_world(package, specs,
        obstacles=_boolean(cfg, 'obstacles') and simulating,
        overhead_camera=overhead_camera, overhead_camera_rate=overhead_rate,
        real_time_factor=1.0 if _boolean(cfg, 'limit_real_time_factor') else 0.0))
    bridge_path.write_text(yaml.safe_dump(bridge_config([s[0] for s in specs],
        clock_topic='/simulation/clock_raw', overhead_camera=overhead_camera)))
    headless = _boolean(cfg, 'headless')
    # Keep display selection isolated even when two simulations share a domain.
    selection_topic = '/cachecons/gui_' + str(os.getpid()) + '/selected_robot'
    # Run gz sim directly: the Humble ros_gz wrapper can orphan server/GUI processes on SIGINT.
    sim_args = ['gz', 'sim', '-s', '-r', '-v', '3', '--force-version', '8']
    if headless:
        sim_args.append('--headless-rendering')
    sim_args.append(str(world_path))
    actions = [
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.pathsep.join(filter(None,
            [str(package / 'models'), os.environ.get('GZ_SIM_RESOURCE_PATH')]))),
        SetEnvironmentVariable('GZ_PARTITION', 'cachecons-' + str(os.getpid())),
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(
            function=lambda context: shutil.rmtree(runtime, ignore_errors=True))])),
        ExecuteProcess(cmd=sim_args, output='screen',
            on_exit=[EmitEvent(event=Shutdown(reason='Gazebo server exited'))]),
        Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
             parameters=[{'config_file': str(bridge_path), 'use_sim_time': True}]),
        Node(package='tf_relay', executable='relay', arguments=[','.join(namespaces)],
             parameters=[{'use_sim_time': False}], output='screen'),
    ]
    actions.append(Node(package='tf_relay', executable='clock_relay',
        parameters=[{'use_sim_time': False, 'rate': 100.0}], output='screen',
        on_exit=[EmitEvent(event=Shutdown(reason='Simulation clock relay exited'))]))
    after_ready = []
    if not headless:
        # Packaged GUI config, never the user's saved one; GZ_PARTITION isolates this run.
        gui = Path(get_package_prefix('cachecons_gazebo_gui')) / 'lib/cachecons_gazebo_gui/cachecons_gazebo_gui'
        gui_args = [str(gui), str(package / 'config/gui_overhead.config'),
                    '--selection-topic', selection_topic]
        for namespace, entity, _ in specs:
            gui_args.extend(['--robot', namespace + '=' + entity])
        # Start after readiness so the server and GUI renderers do not initialise concurrently.
        after_ready.append(ExecuteProcess(cmd=gui_args,
            output='screen', on_exit=[EmitEvent(event=Shutdown(reason='Gazebo GUI exited'))]))
    if simulating:
        manager = Node(package='CacheCons', executable='item_manager', output='screen',
            parameters=[{'use_sim_time': True, 'overhead_camera': overhead_camera}], arguments=[
                '--random_seed', cfg['random_seed'], '--robot_namespaces', ','.join(namespaces)])
        actions.extend([RegisterEventHandler(OnProcessExit(target_action=manager,
            on_exit=[EmitEvent(event=Shutdown(reason='Item manager exited'))])), manager])
        if overhead_camera:
            actions.append(Node(package='CacheCons', executable='overhead_pose', output='screen',
                parameters=[{'use_sim_time': True, 'robot_namespaces': ','.join(namespaces)}]))
    description = (package / 'urdf/turtlebot3_burger.urdf').read_text()
    for index, (namespace, _, _) in enumerate(specs, 1):
        actions.append(GroupAction([PushRosNamespace(namespace),
            SetRemap('/tf', 'tf'), SetRemap('/tf_static', 'tf_static'),
            Node(package='robot_state_publisher', executable='robot_state_publisher',
                 parameters=[{'robot_description': description, 'use_sim_time': True}])]))
        if simulating:
            for executable in (f'CacheCons{index}', *machines):
                after_ready.append(Node(package='CacheCons', executable=executable,
                    namespace=namespace, output='screen', parameters=[{
                        'use_sim_time': True, 'cluster_distance': float(cfg['cluster_distance']),
                        # Seeds each robot's initial cache points as well as the puck layout.
                        'random_seed': int(cfg['random_seed']),
                        # A partial set of machines is driven from outside; do not wait for the others.
                        'wait_for_machines': machines == MACHINES}]))
    if _boolean(cfg, 'state_viewer') and not headless and simulating:
        actions.append(Node(package='cachecons_state_viewer', executable='cachecons_state_viewer',
            parameters=[{'robot_namespaces': namespaces,
                         'selection_topic': selection_topic,
                         'diagram_path': str(Path(get_package_share_directory('cachecons_state_viewer')) / 'images')}]))
    if calibrate != 'none':
        namespace, entity, _ = specs[0]
        after_ready.append(Node(package='CacheCons', executable='camera_calibrator',
            namespace=namespace, output='screen',
            parameters=[{'use_sim_time': True, 'output': calibration_output,
                         'feature': 'marker' if calibrate == 'onboard' else 'aruco'}],
            remappings=[('camera/image_raw', '/overhead_camera/image_raw')]
                if calibrate == 'overhead' else []))
        after_ready.append(Node(package='CacheCons', executable='calibration_grid',
            namespace=namespace, output='screen', arguments=['--robot', entity, '--camera', calibrate],
            parameters=[{'use_sim_time': True}],
            on_exit=[EmitEvent(event=Shutdown(reason='Camera calibration finished'))]))
    readiness = Node(package='CacheCons', executable='simulation_ready',
        parameters=[{'use_sim_time': False, 'robot_namespaces': ','.join(s[0] for s in specs),
                     'wait_for_items': simulating, 'overhead_camera': overhead_camera}], output='screen')
    def ready(event, context):
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(reason='Gazebo sensor/readiness check failed'))]
        return [LogInfo(msg='Harmonic is ready.'), *after_ready]
    actions.extend([RegisterEventHandler(OnProcessExit(target_action=readiness, on_exit=ready)), readiness])
    return actions


def generate_launch_description():
    defaults = dict(num_robots='3', controller='single', domain_id='31', headless='false', state_viewer='true',
        random_seed='0', cluster_distance='0.25',
        obstacles='true', limit_real_time_factor='true',
        overhead_camera='false', overhead_camera_rate='5.0',
        calibrate_camera='none', calibration_output='')
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()],
        SetEnvironmentVariable('ROS_DOMAIN_ID', LaunchConfiguration('domain_id')),
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
        OpaqueFunction(function=_start)])
