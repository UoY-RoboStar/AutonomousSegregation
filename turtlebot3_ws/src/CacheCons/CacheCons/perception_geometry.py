"""Geometry for puck observations (metres, radians, world frame)."""
from dataclasses import dataclass
import math


COLOUR_NAMES = {1: 'red', 2: 'green', 3: 'blue'}
# models/item/meshes/puck.stl: the 60 mm CAD puck.
PUCK_RADIUS = 0.03


def finite_point(point):
    return len(point) == 2 and all(math.isfinite(float(value)) for value in point)


def local_to_world(point, pose):
    """World-frame position of a base-frame point for a robot pose (x, y, yaw)."""
    if not finite_point(point) or len(pose) != 3 or not all(map(math.isfinite, pose)):
        raise ValueError('Point and pose must be finite')
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return (pose[0] + c * point[0] - s * point[1],
            pose[1] + s * point[0] + c * point[1])


def world_to_local(point, pose):
    if not finite_point(point) or len(pose) != 3 or not all(map(math.isfinite, pose)):
        raise ValueError('Point and pose must be finite')
    dx, dy = point[0] - pose[0], point[1] - pose[1]
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return (c * dx + s * dy, -s * dx + c * dy)


@dataclass(frozen=True)
class Detection:
    colour: int
    local: tuple
    pixel: tuple = (0, 0)


@dataclass(frozen=True)
class ObservedCluster:
    cluster_id: int
    colour: int
    members: tuple
    centroid: tuple


def cluster_observations(detections, pose, neighbour_distance=0.25):
    """Group finite detections into same-colour components; IDs are per image, not tracks."""
    if not math.isfinite(neighbour_distance) or neighbour_distance <= 0:
        raise ValueError('neighbour_distance must be positive and finite')
    valid = tuple(d for d in detections if d.colour in COLOUR_NAMES
                  and finite_point(d.local))
    points = tuple(local_to_world(d.local, pose) for d in valid)
    neighbours = [set() for _ in valid]
    for i, first in enumerate(valid):
        for j in range(i + 1, len(valid)):
            if (first.colour == valid[j].colour
                    and math.dist(first.local, valid[j].local) <= neighbour_distance):
                neighbours[i].add(j)
                neighbours[j].add(i)
    remaining = set(range(len(valid)))
    clusters = []
    while remaining:
        stack, members = [min(remaining)], set()
        while stack:
            index = stack.pop()
            if index in members:
                continue
            members.add(index)
            stack.extend(neighbours[index] - members)
        remaining.difference_update(members)
        indices = tuple(sorted(members))
        centroid = tuple(sum(points[i][axis] for i in indices) / len(indices)
                         for axis in (0, 1))
        clusters.append(ObservedCluster(len(clusters) + 1, valid[indices[0]].colour,
                                        indices, centroid))
    return valid, points, tuple(clusters), tuple(len(n) for n in neighbours)


def nearest_reachable_puck(pucks, clusters, pose, colour=None,
                           gripper=(0.15, 0.0), reach=0.10):
    """Return the nearest reachable ROS puck, restricted to one colour when given."""
    if colour is not None and colour not in COLOUR_NAMES:
        return None
    cluster_colours = {int(c.clusterid[0]): int(c.clustertype[0]) for c in clusters
                       if len(c.clusterid) and len(c.clustertype)}
    candidates = []
    for puck in pucks:
        if not len(puck.clusterid) or (colour is not None
                                        and cluster_colours.get(int(puck.clusterid[0])) != colour):
            continue
        point = (puck.positionx, puck.positiony)
        if not finite_point(point):
            continue
        local = world_to_local(point, pose)
        distance = math.dist(local, gripper)
        if local[0] >= 0 and distance <= reach:
            candidates.append((distance, puck))
    return min(candidates, key=lambda candidate: candidate[0])[1] if candidates else None


def pose_for_image(history, stamp, tolerance=0.3):
    """PoseO has no header: match its ROS receipt time to the image timestamp."""
    if not history:
        return None
    timestamp, pose = min(history, key=lambda record: abs(record[0] - stamp))
    return pose if abs(timestamp - stamp) <= tolerance else None
