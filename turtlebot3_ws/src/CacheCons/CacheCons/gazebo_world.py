"""Build the Gazebo Harmonic world SDF and ros_gz bridge config from the package assets."""
from pathlib import Path
import xml.etree.ElementTree as ET


def read_sdf(path):
    return ET.parse(path).getroot()


def value(parent, name, text):
    element = parent.find(name)
    if element is None:
        element = ET.SubElement(parent, name)
    element.text = str(text)
    return element


def system(parent, library, name, **parameters):
    plugin = ET.SubElement(parent, 'plugin', filename='gz-sim-' + library + '-system',
                           name='gz::sim::systems::' + name)
    for key, text in parameters.items():
        value(plugin, key, text)
    return plugin


CAMERA_RATE = 10.0
JOINT_STATE_RATE = 30.0
PHYSICS_STEP = 0.001


def sensors(model, namespace, camera_rate=CAMERA_RATE):
    for sensor in model.iter('sensor'):
        kind = sensor.get('type')
        if kind == 'gpu_lidar':
            topic, frame = 'scan', 'base_scan'
        elif kind == 'camera':
            topic, frame = 'camera/image_raw', 'camera_rgb_optical_frame'
            value(sensor, 'update_rate', camera_rate)
            value(sensor.find('camera'), 'camera_info_topic', f'/{namespace}/camera/camera_info')
        elif kind == 'imu':
            topic, frame = 'imu', 'imu_link'
        else:
            continue
        value(sensor, 'topic', f'/{namespace}/{topic}')
        value(sensor, 'gz_frame_id', frame)


# The real robots' tag mount: a white plate on standoffs and clear end panels above the
# lidar, carrying ArUco tag N (DICT_4X4_100, 45 mm) with its top edge facing the robot's right.
TAG_MOUNT = """<visual name="tag_plate">
  <pose>-0.032 0 0.2065 0 0 0</pose>
  <geometry><box><size>0.138 0.058 0.003</size></box></geometry>
  <material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse></material>
</visual>
<visual name="aruco_tag">
  <pose>-0.032 0 0.2081 0 0 3.1415927</pose>
  <geometry><plane><normal>0 0 1</normal><size>0.0525 0.0525</size></plane></geometry>
  <material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse><specular>0 0 0 1</specular>
    <pbr><metal><albedo_map>model://burger/materials/textures/aruco_{tag}.png</albedo_map>
      <metalness>0</metalness><roughness>1</roughness></metal></pbr></material>
</visual>
""" + "".join(f"""<visual name="tag_standoff_{index}">
  <pose>{-0.032 + x} {y} 0.1775 0 0 0</pose>
  <geometry><box><size>0.006 0.006 0.055</size></box></geometry>
  <material><ambient>0.7 0.7 0.72 1</ambient><diffuse>0.7 0.7 0.72 1</diffuse></material>
</visual>
""" for index, (x, y) in enumerate([(.063, .023), (.063, -.023), (-.063, .023), (-.063, -.023)])
) + "".join(f"""<visual name="tag_panel_{index}">
  <pose>{-0.032 + x} 0 0.1775 0 0 0</pose>
  <geometry><box><size>0.003 0.058 0.055</size></box></geometry>
  <material><ambient>0.9 0.95 1 0.15</ambient><diffuse>0.9 0.95 1 0.15</diffuse></material>
  <transparency>0.85</transparency>
  <cast_shadows>false</cast_shadows>
</visual>
""" for index, x in enumerate([.0675, -.0675]))


def robot_model(package_dir, namespace, entity, pose):
    models = Path(package_dir) / 'models'
    model = read_sdf(models / 'burger/model.sdf').find('model')
    model.set('name', entity)
    value(model, 'pose', f"{pose['x']} {pose['y']} 0.01 0 0 {pose['yaw']}")
    sensors(model, namespace)
    model.find("link[@name='base_link']").extend(ET.fromstring(
        '<visuals>' + TAG_MOUNT.replace('{tag}', entity.removeprefix('robot')) + '</visuals>'))
    system(model, 'diff-drive', 'DiffDrive', left_joint='wheel_left_joint',
           right_joint='wheel_right_joint', wheel_separation=0.160, wheel_radius=0.033,
           topic=f'/{namespace}/cmd_vel', odom_topic=f'/{namespace}/odom',
           tf_topic=f'/{namespace}/tf', frame_id='odom',
           child_frame_id='base_footprint', odom_publish_frequency=30,
           # Wheel = v +/- w*b/2, so each wheel stays under 1 m/s^2: 0.8 + 2*0.08 = 0.96.
           min_linear_acceleration=-0.8, max_linear_acceleration=0.8,
           min_angular_acceleration=-2.0, max_angular_acceleration=2.0)
    system(model, 'joint-state-publisher', 'JointStatePublisher',
           topic=f'/{namespace}/joint_states', update_rate=JOINT_STATE_RATE)
    return model


def build_world(package_dir, specs, *, obstacles=True, overhead_camera=False,
                overhead_camera_rate=5.0, real_time_factor=1.0):
    package_dir = Path(package_dir)
    root = read_sdf(package_dir / 'worlds/arena.world')
    world = root.find('world')
    physics = ET.SubElement(world, 'physics', name='dart', type='ignored')
    value(physics, 'max_step_size', PHYSICS_STEP)
    value(physics, 'real_time_factor', real_time_factor)
    system(world, 'physics', 'Physics')
    system(world, 'user-commands', 'UserCommands')
    system(world, 'scene-broadcaster', 'SceneBroadcaster')
    system(world, 'sensors', 'Sensors', render_engine='ogre2')
    system(world, 'imu', 'Imu')
    if not obstacles:
        for model in list(world.findall('model')):
            if model.get('name', '').startswith(('box', 'cylinder')):
                world.remove(model)
    for namespace, entity, pose in specs:
        world.append(robot_model(package_dir, namespace, entity, pose))
    if overhead_camera:
        camera = read_sdf(package_dir / 'models/overhead_camera/model.sdf').find('model')
        sensors(camera, 'overhead_camera', overhead_camera_rate)
        world.append(camera)
    return ET.tostring(root, encoding='unicode')


def bridge_config(namespaces, clock_topic='/clock', overhead_camera=False):
    bridges = [dict(ros_topic_name=clock_topic, gz_topic_name='/world/cachecons/clock',
                    ros_type_name='rosgraph_msgs/msg/Clock', gz_type_name='gz.msgs.Clock',
                    direction='GZ_TO_ROS')]
    types = [('cmd_vel', 'geometry_msgs/msg/Twist', 'Twist', 'ROS_TO_GZ'),
             ('odom', 'nav_msgs/msg/Odometry', 'Odometry', 'GZ_TO_ROS'),
             ('tf', 'tf2_msgs/msg/TFMessage', 'Pose_V', 'GZ_TO_ROS'),
             ('joint_states', 'sensor_msgs/msg/JointState', 'Model', 'GZ_TO_ROS'),
             ('scan', 'sensor_msgs/msg/LaserScan', 'LaserScan', 'GZ_TO_ROS'),
             ('imu', 'sensor_msgs/msg/Imu', 'IMU', 'GZ_TO_ROS'),
             ('camera/image_raw', 'sensor_msgs/msg/Image', 'Image', 'GZ_TO_ROS'),
             ('camera/camera_info', 'sensor_msgs/msg/CameraInfo', 'CameraInfo', 'GZ_TO_ROS')]
    for namespace in namespaces:
        for topic, ros_type, gz_type, direction in types:
            bridges.append(dict(ros_topic_name=f'/{namespace}/{topic}',
                                gz_topic_name=f'/{namespace}/{topic}',
                                ros_type_name=ros_type, gz_type_name='gz.msgs.' + gz_type,
                                direction=direction))
    for topic, ros_type, gz_type in ([('image_raw', 'Image', 'Image'),
                                    ('camera_info', 'CameraInfo', 'CameraInfo')] if overhead_camera else []):
        bridges.append(dict(ros_topic_name='/overhead_camera/' + topic,
                            gz_topic_name='/overhead_camera/camera/' + topic,
                            ros_type_name='sensor_msgs/msg/' + ros_type,
                            gz_type_name='gz.msgs.' + gz_type, direction='GZ_TO_ROS'))
    return bridges
