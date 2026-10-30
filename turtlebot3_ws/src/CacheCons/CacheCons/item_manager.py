import os
import sys
import math
import random
import argparse
import json
from enum import Enum
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import Bool, String
from rclpy.qos import QoSProfile, DurabilityPolicy
from .gazebo_backend import GazeboBackend, compose_pose
from geometry_msgs.msg import Pose
from interfaces.msg import PoseO

from tf_transformations import euler_from_quaternion


import yaml

DROP_OFFSET = 0.2
PICKUP_OFFSET = 0.15
PICKUP_RADIUS = 0.10
INITIAL_ROBOT_PUCK_CLEARANCE = 0.30

class Colour(Enum):
    RED = 1
    GREEN = 2
    BLUE = 3


class Item():
    def __init__(self, x, y, colour, cluster_id):
        self.x = x
        self.y = y
        self.colour = colour
        self.cluster_id = cluster_id

    def __repr__(self):
        return f'({self.x}, {self.y})'


class Cluster():
    def __init__(self, x, y, colour):
        self.x = x
        self.y = y
        self.colour = colour

    def __repr__(self):
        return f'({self.x}, {self.y})'


class Robot():
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.item_held = None
        self.previous_item_held = None

    def __repr__(self):
        return f'({self.x}, {self.y}, {self.item_held})'


class ItemManager(Node):

    def __init__(self, args):
        super().__init__('item_manager')

        parser = argparse.ArgumentParser()

        parser.add_argument('--random_seed', type=int, required=True, metavar='RANDOM_SEED', help='Random seed')
        parser.add_argument(
            '--robot_namespaces',
            required=True,
            help='Comma-separated ROS namespaces, in Gazebo robot order')
        self.args = parser.parse_args(args[1:])

        self.robot_namespaces = [
            namespace.strip().strip('/')
            for namespace in self.args.robot_namespaces.split(',')
            if namespace.strip()
        ]
        if not 1 <= len(self.robot_namespaces) <= 3:
            parser.error('--robot_namespaces must contain between one and three names')

        self.get_logger().info(
            f'Robot namespaces: {", ".join(self.robot_namespaces)}')
        random.seed(self.args.random_seed)
        # Same start table as the launch, so random pucks avoid each robot's body and gripper.
        with open(os.path.join(get_package_share_directory('CacheCons'),
                               'config', 'initial_poses.yaml')) as pose_file:
            start_poses = yaml.safe_load(pose_file)[len(self.robot_namespaces)]
        self.initial_robot_positions = [(pose['x'], pose['y'])
                                        for pose in start_poses.values()]
        # With the overhead camera, robot poses come from its ArUco tags instead.
        self.overhead_camera = bool(self.declare_parameter('overhead_camera', False).value)

        self.first_run = True

        self.clusters = {}
        self.robots = {}
        self.items = {}
        self.last_snapshot_at = -math.inf

        self.cluster_counter = 0
        self.item_counter = 0

        self.item_models = {}

        models_path = os.path.join(get_package_share_directory("CacheCons"), "models")
        itemR_model_path = os.path.join(models_path, "item", "modelR.sdf")

        itemG_model_path = os.path.join(models_path, "item", "modelG.sdf")

        itemB_model_path = os.path.join(models_path, "item", "modelB.sdf")
        

        treeR = ET.parse(itemR_model_path)
        rootR = treeR.getroot()

        treeG = ET.parse(itemG_model_path)
        rootG = treeG.getroot()

        treeB = ET.parse(itemB_model_path)
        rootB = treeB.getroot()

        self.item_models[Colour.RED] = ET.tostring(rootR, encoding='unicode')

        self.item_models[Colour.GREEN] = ET.tostring(rootG, encoding='unicode')

        self.item_models[Colour.BLUE] = ET.tostring(rootB, encoding='unicode')

        timer_callback_group = MutuallyExclusiveCallbackGroup()
        publisher_callback_group = ReentrantCallbackGroup()
        # All robots share the timer's group, so puck ownership changes are serialized.
        robot_callback_groups = [timer_callback_group for _ in self.robot_namespaces]

        self.gazebo = GazeboBackend(
            [f'robot{i}' for i in range(1, len(self.robot_namespaces) + 1)])
        self.ready_publisher = self.create_publisher(
            Bool, '/simulation/items_ready',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

        self.timer = self.create_timer(0.1, self.control_loop, callback_group=timer_callback_group)

        self.item_event_publisher = self.create_publisher(String, '/simulation/item_events', 10)
        # Validation only: controllers must learn caches from their cameras.
        self.snapshot_publisher = self.create_publisher(String, '/simulation/clustering_ground_truth', 10)

        pick_callbacks = [
            self.PickPuckCallback1,
            self.PickPuckCallback2,
            self.PickPuckCallback3,
        ]
        deposit_callbacks = [
            self.depositPuckCallback1,
            self.depositPuckCallback2,
            self.depositPuckCallback3,
        ]
        for index, (namespace, callback_group) in enumerate(
                zip(self.robot_namespaces, robot_callback_groups), start=1):
            prefix = f'/{namespace}'
            setattr(
                self,
                f'PuckCarried_pub{index}',
                self.create_publisher(
                    Bool,
                    f'{prefix}/PuckCarried',
                    10,
                    callback_group=callback_group))
            setattr(
                self,
                f'Publish_Pose_O{index}',
                self.create_publisher(
                    PoseO,
                    f'{prefix}/PoseO',
                    10,
                    callback_group=callback_group))
            setattr(
                self,
                f'PickPuck{index}',
                self.create_subscription(
                    Bool,
                    f'{prefix}/pick',
                    pick_callbacks[index - 1],
                    10,
                    callback_group=callback_group))
            setattr(
                self,
                f'depositPuck{index}',
                self.create_subscription(
                    Bool,
                    f'{prefix}/place',
                    deposit_callbacks[index - 1],
                    10,
                    callback_group=callback_group))

    def pickup_blocked_by_released_item(self, robot, pickup_radius):
        """Keep a robot from immediately collecting the puck it just dropped."""
        released_item = robot.previous_item_held
        if released_item is None:
            return False

        item = self.items.get(released_item)
        if item is not None and math.dist(
                (robot.x, robot.y), (item.x, item.y)) <= pickup_radius:
            return True

        robot.previous_item_held = None
        return False


    def _item_event(self, kind, robot_id, item_id):
        item = self.items[item_id]
        same, other = [], []
        for identifier, candidate in self.items.items():
            if identifier == item_id or self.item_is_held(identifier):
                continue
            distances = same if candidate.colour == item.colour else other
            distances.append(math.dist((item.x, item.y), (candidate.x, candidate.y)))
        payload = dict(kind=kind, robot=robot_id, item=item_id,
                       colour=item.colour.name, x=item.x, y=item.y,
                       nearest_same=min(same) if same else None,
                       nearest_other=min(other) if other else None,
                       time=self.get_clock().now().nanoseconds * 1e-9)
        self.item_event_publisher.publish(String(data=json.dumps(payload)))
        self.get_logger().info('ITEM_EVENT ' + json.dumps(payload))

    def _pick_puck(self, robot_id, message):
        if not message.data or robot_id not in self.robots:
            return
        robot = self.robots[robot_id]
        if robot.item_held is not None:
            return
        pose = self.gazebo.get_pose(robot_id)
        robot.x, robot.y = pose.position.x, pose.position.y
        q = pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        if self.pickup_blocked_by_released_item(robot, PICKUP_OFFSET + PICKUP_RADIUS):
            return
        gripper = (robot.x + PICKUP_OFFSET * math.cos(yaw),
                   robot.y + PICKUP_OFFSET * math.sin(yaw))
        candidates = [
            (math.dist(gripper, (item.x, item.y)), item_id)
            for item_id, item in self.items.items()
            if not self.item_is_held(item_id)
        ]
        candidates = [candidate for candidate in candidates if candidate[0] <= PICKUP_RADIUS]
        if candidates:
            robot.item_held = min(candidates)[1]
            self._item_event('pick', robot_id, robot.item_held)

    def PickPuckCallback1(self, msg):
        ItemManager._pick_puck(self, 'robot1', msg)

    def PickPuckCallback2(self, msg):
        ItemManager._pick_puck(self, 'robot2', msg)

    def PickPuckCallback3(self, msg):
        ItemManager._pick_puck(self, 'robot3', msg)

    def _deposit_puck(self, robot_id, message):
        if not message.data or robot_id not in self.robots:
            return
        robot = self.robots[robot_id]
        item_id = robot.item_held
        if item_id is None or item_id not in self.items:
            return
        robot_pose = self.gazebo.get_pose(robot_id)
        q = robot_pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        pose = Pose()
        pose.position.x = robot_pose.position.x + DROP_OFFSET * math.cos(yaw)
        pose.position.y = robot_pose.position.y + DROP_OFFSET * math.sin(yaw)
        # Pucks rest on the floor, as spawn_item places them.
        pose.position.z = 0.0
        pose.orientation.w = 1.0
        self.gazebo.set_pose(item_id, 'world', pose)
        item = self.items[item_id]
        item.x, item.y = pose.position.x, pose.position.y
        robot.previous_item_held = item_id
        robot.item_held = None
        self._item_event('release', robot_id, item_id)

    def depositPuckCallback1(self, msg):
        ItemManager._deposit_puck(self, 'robot1', msg)

    def depositPuckCallback2(self, msg):
        ItemManager._deposit_puck(self, 'robot2', msg)

    def depositPuckCallback3(self, msg):
        ItemManager._deposit_puck(self, 'robot3', msg)


    def generate_cluster_location(self, colour):
        while True:
            y = random.randint(-2, 2)
            match colour:
                case Colour.RED:
                    x = random.randint(-2, -1)
                case Colour.GREEN:
                    x = random.randint(-1, 1)
                case Colour.BLUE:
                    x = random.randint(1, 2)
                case _:
                    pass

            # Invalid if it clashes with an obstacle
            if (x == 1 and y == 1) or (x == 1 and y == -1) or (x == -1 and y == 1) or (x == -1 and y == -1):
                continue

            # Invalid if it clashes with an existing cluster
            for cluster in self.clusters.values():
                if (cluster.x == x and cluster.y == y) or math.dist((cluster.x, cluster.y), (x, y)) <= 1.0:
                    break
            else:                    
                return x, y

    
    def generate_item_position(self, cluster_id):
        while True:

            radius = random.uniform(0, 0.5)
            angle = math.radians(random.uniform(0, 360))

            x = self.clusters[cluster_id].x + round(radius * math.cos(angle), 2)
            y = self.clusters[cluster_id].y + round(radius * math.sin(angle), 2)

            if any(math.dist((x, y), position) < INITIAL_ROBOT_PUCK_CLEARANCE
                   for position in self.initial_robot_positions):
                continue

            for item in self.items.values():
                if item.cluster_id == cluster_id and math.dist((item.x, item.y), (x, y)) < 0.3:
                    break
            else:
                return x, y


    def spawn_item(self, name, x, y, colour, z = 0.0):
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = float(x), float(y), float(z)
        pose.orientation.w = 1.0
        self.item_counter += 1
        self.gazebo.spawn(name, self.item_models[colour], pose)

    def item_is_held(self, item_id):
        return any(robot.item_held == item_id for robot in self.robots.values())

    def place_carried_item(self, item_id, robot_pose, local_pose):
        # The bookkeeping table is always world-relative, including while held.
        world_pose = compose_pose(robot_pose, local_pose)
        self.gazebo.set_pose(item_id, 'world', world_pose)
        self.items[item_id].x = world_pose.position.x
        self.items[item_id].y = world_pose.position.y

    def control_loop(self):
        if self.first_run:
            self.first_run = False

            self.get_logger().info('Spawning random items...')
            for i in range(6):

                valid = False
                colour = random.choice(list(Colour))

                while not valid:

                    count = 0

                    for cluster in self.clusters.values():
                        if colour == cluster.colour:
                            count += 1
                
                    if count < 2:
                        valid = True
                    else:
                        colour = random.choice(list(Colour))

                cluster_id = "cluster" + str(self.cluster_counter)
                self.cluster_counter += 1

                x, y = self.generate_cluster_location(colour)

                self.clusters[cluster_id] = Cluster(x, y, colour)

                for j in range(random.randint(3, 5)):

                    x, y = self.generate_item_position(cluster_id)
                    colour = random.choice([Colour.RED, Colour.GREEN, Colour.BLUE])
                    item_id = "item" + str(self.item_counter)
                    self.items[item_id] = Item(x, y, colour, cluster_id)

                    self.spawn_item(item_id, x, y, colour)
            self.ready_publisher.publish(Bool(data=True))


        for model_name in self.gazebo.models():
            if "robot" in model_name:
                if model_name not in self.robots:
                    self.robots[model_name] = Robot(0, 0)

        for robot_id, robot in self.robots.items():

            robot_pose = self.gazebo.get_pose(robot_id)
            robot_position = robot_pose.position
            robot_orientation = robot_pose.orientation

            robot.x = robot_position.x
            robot.y = robot_position.y
            (roll, pitch, yaw) = euler_from_quaternion([robot_orientation.x,
                                                    robot_orientation.y,
                                                    robot_orientation.z,
                                                    robot_orientation.w])
            robot.theta = yaw
            index = int(robot_id.removeprefix('robot'))
            if not self.overhead_camera:
                msg = PoseO()
                msg.poseo = (robot.x, robot.y, robot.theta)
                getattr(self, f'Publish_Pose_O{index}').publish(msg)
            getattr(self, f'PuckCarried_pub{index}').publish(
                Bool(data=robot.item_held is not None))
            if robot.item_held is not None:
                # The same gripper point is used for capture and carry.
                pose = Pose()
                pose.position.x = PICKUP_OFFSET
                pose.position.z = 0.15
                pose.orientation.w = 1.0
                self.place_carried_item(
                    robot.item_held, robot_pose, pose)
            robot.roll, robot.pitch = roll, pitch

        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.last_snapshot_at >= 1.0:
            self.last_snapshot_at = now
            snapshot = dict(time=now, items=[
                dict(id=key, colour=item.colour.value, x=item.x, y=item.y,
                     held=self.item_is_held(key))
                for key, item in self.items.items()], robots={
                key: dict(x=robot.x, y=robot.y, roll=robot.roll, pitch=robot.pitch)
                for key, robot in self.robots.items()})
            self.snapshot_publisher.publish(String(data=json.dumps(snapshot)))


def main(args=sys.argv):

    rclpy.init(args=args)

    args_without_ros = rclpy.utilities.remove_ros_args(args)

    node = ItemManager(args_without_ros)

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        executor.shutdown()
        node.gazebo.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
