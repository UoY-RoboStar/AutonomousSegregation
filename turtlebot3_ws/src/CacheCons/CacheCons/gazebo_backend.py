"""gz-transport client for spawning models and reading/setting world-relative poses.

Every request has a timeout; failed requests raise.
"""
import copy
import math
import importlib
import multiprocessing
import threading
import time


def compose_pose(parent, child):
    """Compose two poses (including quaternion rotation), without ROS imports."""
    def quaternion(q):
        values = [q.x, q.y, q.z, q.w]
        length = math.sqrt(sum(x*x for x in values))
        return [x/length for x in values] if length else [0, 0, 0, 1]
    x, y, z, w = quaternion(parent.orientation)
    u, v, s, t = quaternion(child.orientation)
    px, py, pz = child.position.x, child.position.y, child.position.z
    result = copy.deepcopy(child)
    result.position.x = parent.position.x + (1-2*y*y-2*z*z)*px + (2*x*y-2*z*w)*py + (2*x*z+2*y*w)*pz
    result.position.y = parent.position.y + (2*x*y+2*z*w)*px + (1-2*x*x-2*z*z)*py + (2*y*z-2*x*w)*pz
    result.position.z = parent.position.z + (2*x*z-2*y*w)*px + (2*y*z+2*x*w)*py + (1-2*x*x-2*y*y)*pz
    result.orientation.x = w*u + x*t + y*s - z*v
    result.orientation.y = w*v - x*s + y*t + z*u
    result.orientation.z = w*s + x*v - y*u + z*t
    result.orientation.w = w*t - x*u - y*v - z*s
    return result


def _service_worker(connection):
    """Serve blocking gz requests from a separate process that has no subscriptions."""
    from gz.transport13 import Node
    import gz.msgs10  # Register the generated messages' gz.msgs module alias.
    node = Node()
    try:
        while True:
            job = connection.recv()
            if job is None:
                return
            service, payload, req_module, req_name, rep_module, rep_name, timeout = job
            try:
                request_type = getattr(importlib.import_module(req_module), req_name)
                response_type = getattr(importlib.import_module(rep_module), rep_name)
                request = request_type()
                request.ParseFromString(payload)
                # request_raw holds the GIL; in-process, a pose callback would block the
                # receive thread this reply needs.
                ok, response = node.request(service, request, request_type,
                                            response_type, timeout)
                connection.send((ok, response.SerializeToString(), None))
            except Exception as exc:
                connection.send((False, b'', str(exc)))
    except EOFError:
        pass
    finally:
        connection.close()


class GazeboServiceClient:
    def __init__(self):
        context = multiprocessing.get_context('spawn')
        self.connection, child = context.Pipe()
        self.lock = threading.Lock()
        self.failed = False
        self.worker = context.Process(target=_service_worker, args=(child,), daemon=True)
        self.worker.start()
        child.close()

    def request(self, service, request, request_type, response_type, timeout):
        with self.lock:
            if self.failed or not self.worker.is_alive():
                raise RuntimeError('Gazebo service worker exited')
            self.connection.send((service, request.SerializeToString(),
                request_type.__module__, request_type.__name__,
                response_type.__module__, response_type.__name__, timeout))
            if not self.connection.poll(timeout / 1000 + 5):
                # Do not reuse the pipe: a late reply must not satisfy a later job.
                self.failed = True
                self.worker.terminate()
                raise RuntimeError('Gazebo service worker did not respond')
            ok, payload, error = self.connection.recv()
            if error:
                raise RuntimeError('Gazebo service worker: ' + error)
            response = response_type()
            response.ParseFromString(payload)
            return ok, response

    def close(self):
        with self.lock:
            if self.worker.is_alive():
                self.connection.send(None)
                self.worker.join(timeout=1)
            if self.worker.is_alive():
                self.worker.terminate()
                self.worker.join(timeout=1)
            self.connection.close()


class GazeboBackend:
    def __init__(self, robot_names, world='cachecons', timeout_ms=3000):
        from gz.transport13 import Node
        from gz.msgs10.pose_v_pb2 import Pose_V
        self.node = Node()
        self.services = GazeboServiceClient()
        self.world = world
        self.timeout_ms = timeout_ms
        self.known_models = set(robot_names)
        self.poses = {}
        self.condition = threading.Condition()
        self.topic = f'/world/{world}/pose/info'
        if not self.node.subscribe(Pose_V, self.topic, self._poses):
            raise RuntimeError('Could not subscribe to Gazebo poses: ' + self.topic)

    def _poses(self, message):
        from geometry_msgs.msg import Pose
        with self.condition:
            for source in message.pose:
                # SceneBroadcaster also publishes link/visual-local poses. Only
                # accept explicitly known top-level models, whose pose is world-relative.
                if source.name not in self.known_models:
                    continue
                pose = Pose()
                for name in ('x', 'y', 'z'):
                    setattr(pose.position, name, getattr(source.position, name))
                for name in ('x', 'y', 'z', 'w'):
                    setattr(pose.orientation, name, getattr(source.orientation, name))
                self.poses[source.name] = (pose, time.monotonic())
            self.condition.notify_all()

    def _pose(self, name):
        deadline = time.monotonic() + self.timeout_ms / 1000
        with self.condition:
            while True:
                entry = self.poses.get(name)
                if entry and time.monotonic() - entry[1] < 1.0:
                    return copy.deepcopy(entry[0])
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise RuntimeError(f'Gazebo pose missing or stale for {name}')
                self.condition.wait(min(remaining, 0.1))

    def _request(self, suffix, request, response_type):
        ok, response = self.services.request(f'/world/{self.world}/{suffix}', request,
                                         type(request), response_type, self.timeout_ms)
        if not ok or not response.data:
            raise RuntimeError(f'Gazebo {suffix} failed or timed out '
                               f'(transport={ok}, accepted={response.data})')

    @staticmethod
    def _copy_pose(source, target):
        for name in ('x', 'y', 'z'):
            setattr(target.position, name, getattr(source.position, name))
        values = [getattr(source.orientation, n) for n in ('x', 'y', 'z', 'w')]
        length = math.sqrt(sum(v*v for v in values))
        if not length:
            values, length = [0, 0, 0, 1], 1
        for name, number in zip(('x', 'y', 'z', 'w'), values):
            setattr(target.orientation, name, number / length)

    def spawn(self, name, xml, pose):
        from gz.msgs10.entity_factory_pb2 import EntityFactory
        from gz.msgs10.boolean_pb2 import Boolean
        request = EntityFactory()
        request.name, request.sdf, request.allow_renaming = name, xml, False
        self._copy_pose(pose, request.pose)
        with self.condition:
            self.known_models.add(name)
        self._request('create', request, Boolean)
        self._pose(name)  # Accepted creation is not yet proof of a spawned entity.

    def models(self):
        with self.condition:
            names = [name for name, (_, stamp) in self.poses.items()
                     if time.monotonic() - stamp < 1.0]
        return names

    def get_pose(self, name):
        return self._pose(name)

    def set_pose(self, name, reference_frame, pose):
        from gz.msgs10.pose_pb2 import Pose
        from gz.msgs10.boolean_pb2 import Boolean
        if reference_frame not in ('', 'world'):
            pose = compose_pose(self._pose(reference_frame), pose)
        request = Pose()
        request.name = name
        self._copy_pose(pose, request)
        self._request('set_pose', request, Boolean)

    def close(self):
        self.services.close()
        self.node.unsubscribe(self.topic)
