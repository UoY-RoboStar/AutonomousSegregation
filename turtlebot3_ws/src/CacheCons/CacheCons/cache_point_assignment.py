"""CachePointAssignment state machine: remembers the cache point of each object type."""
from enum import Enum

import rclpy
from rclpy.node import Node
import copy
import math
import random

from interfaces.msg import CachePointsCC, VisibleClustersCC
from std_msgs.msg import Int32

from .helpers import zero_cluster


class CPA_State(Enum):
    DetectCluster = 0
    CalculateL = 1
    DecideNewCachePoint = 2
    UpdateCache = 3
    UpdateCurrentCache = 4
    CheckSeparation = 5
    ClearConflicts = 6


def distance(x1, x2):
    return math.sqrt((x1[0] - x2[0])**2 + (x1[1] - x2[1])**2)


# Where initial cache points may be drawn: the arena less a margin from its walls.
ARENA_X = (-3.3, 2.3)
ARENA_Y = (-2.3, 2.3)


def initial_caches(seed, robot_index, types=3, separation=2.0):
    """The model's initialCaches parameter: one cache per type with size 0 at an arbitrary point.

    Vardy et al. (2014) start from an arbitrary distribution of caches. The points are drawn
    from the launch's random seed, differently for each robot, and at least `separation`
    apart so that no two types start at a shared point.
    """
    rng = random.Random(f'{seed}-{robot_index}')
    points = []
    while len(points) < types:
        p = (rng.uniform(*ARENA_X), rng.uniform(*ARENA_Y))
        if all(distance(p, q) >= separation for q in points):
            points.append(p)
    caches = []
    for t, p in enumerate(points, start=1):
        c = zero_cluster()
        c.clustertype = [t]
        c.centroid = p
        caches.append(c)
    return caches


class CPA():
    # Cache separation (Vardy et al. 2014, section 2.6.2): caches of different types stay at
    # least this far apart. A candidate cache within it of another type's cache is adopted only
    # if it is larger, and the conflicting caches are then reset to their initial points.
    separation = 2.0
    # A cache's size is refreshed only from the cluster seen at its own position (equation 5 of
    # the paper); the cache point itself stays where it was adopted.
    cache_radius = 0.5
    types = 3
    exec = False
    j = 0
    counter = 0
    k = 1
    blocked = False
    L = zero_cluster()
    data = VisibleClustersCC()
    initial = initial_caches(0, 0)
    m = [copy.deepcopy(c) for c in initial]


class CachePointAssignmentMachine:
    def __init__(self, logger, timer_period=0.1, initial=None):
        self.logger = logger
        self.timer_period = timer_period
        self.CPA_state = CPA_State.DetectCluster
        self.CPA = CPA()
        if initial is not None:
            self.CPA.initial = initial
            self.CPA.types = len(initial)
            self.CPA.m = [copy.deepcopy(c) for c in initial]
        logger.info('initial cache points ' + ', '.join(
            f'type {c.clustertype[0]} at ({c.centroid[0]:.2f}, {c.centroid[1]:.2f})' for c in self.CPA.m))
        # Events in
        self.VisibleClustersCA = [False, VisibleClustersCC()]
        self.CurrentTypeCA = [False, 0]
        # Events out: both carry CPA.m, the remembered cache points.
        self.CachePointsCC = [False, self.CPA.m]
        self.CachePointsTW = [False, self.CPA.m]

    def get_logger(self):
        return self.logger

    def CachePointAssignment(self):
        while self.CPA.exec == False:
            match self.CPA_state:
                case CPA_State.DetectCluster:
                    if self.CurrentTypeCA[0] == True:
                        self.CPA.j = self.CurrentTypeCA[1]
                    if self.VisibleClustersCA[0] == False:
                        self.CPA_state = CPA_State.DetectCluster
                        self.CPA.exec = True
                    if self.VisibleClustersCA[0] == True:
                        self.CPA.data = self.VisibleClustersCA[1]
                        self.CPA.counter = 1
                        self.CPA.L.clustersize[0] = 0
                        self.CPA_state = CPA_State.CalculateL
                case CPA_State.CalculateL:
                    if self.CPA.counter <= self.CPA.data.m:
                        if (self.CPA.data.clusterdata[self.CPA.counter-1].clustersize[0] > self.CPA.L.clustersize[0]):
                            self.CPA.L = self.CPA.data.clusterdata[self.CPA.counter-1]
                    if self.CPA.counter <= self.CPA.data.m:
                        self.CPA_state = CPA_State.CalculateL
                        self.CPA.counter += 1
                    elif self.CPA.counter > self.CPA.data.m:
                        self.CPA_state = CPA_State.DecideNewCachePoint
                case CPA_State.DecideNewCachePoint:
                    if self.CurrentTypeCA[0] == True and (self.CPA.L.clustersize[0] > self.CPA.m[self.CPA.L.clustertype[0]-1].clustersize[0]):
                        self.CPA.j = self.CurrentTypeCA[1]
                        self.CPA.k = 1
                        self.CPA.blocked = False
                        self.CPA_state = CPA_State.CheckSeparation
                    elif self.CPA.L.clustersize[0] <= self.CPA.m[self.CPA.L.clustertype[0] -1].clustersize[0]:
                        self.CPA.counter = 0
                        self.CPA_state = CPA_State.UpdateCurrentCache
                    else:
                        self.CPA_state = CPA_State.DetectCluster
                        self.CPA.exec = True
                case CPA_State.CheckSeparation:
                    # The candidate L is blocked by a cache of another type within separation
                    # that is at least as large.
                    L, other = self.CPA.L, self.CPA.m[self.CPA.k - 1]
                    if (self.CPA.k != L.clustertype[0] and other.clustersize[0] >= L.clustersize[0]
                            and distance(other.centroid, L.centroid) < self.CPA.separation):
                        self.CPA.blocked = True
                    if self.CPA.k < self.CPA.types:
                        self.CPA.k += 1
                    elif not self.CPA.blocked:
                        self.CPA_state = CPA_State.UpdateCache
                    else:
                        self.CPA.counter = 0
                        self.CPA_state = CPA_State.UpdateCurrentCache
                case CPA_State.UpdateCache:
                    L = copy.deepcopy(self.CPA.L)
                    self.CPA.m[L.clustertype[0] - 1] = L
                    self.CPA.k = 1
                    self.CPA_state = CPA_State.ClearConflicts
                case CPA_State.ClearConflicts:
                    # The smaller caches of other types within separation of the new cache go
                    # back to their initial points, with size 0.
                    L, other = self.CPA.L, self.CPA.m[self.CPA.k - 1]
                    if (self.CPA.k != L.clustertype[0]
                            and distance(other.centroid, L.centroid) < self.CPA.separation):
                        self.CPA.m[self.CPA.k - 1] = copy.deepcopy(self.CPA.initial[self.CPA.k - 1])
                    if self.CPA.k < self.CPA.types:
                        self.CPA.k += 1
                    else:
                        self.CPA.counter = 0
                        self.CPA_state = CPA_State.UpdateCurrentCache
                case CPA_State.UpdateCurrentCache:
                    if self.CPA.counter <= self.CPA.data.m - 1:
                        C = self.CPA.data.clusterdata[self.CPA.counter]
                        cache = self.CPA.m[self.CPA.j - 1]
                        if (cache.clustersize[0] > 0 and C.clustertype[0] == self.CPA.j
                                and distance(C.centroid, cache.centroid) < self.CPA.cache_radius
                                and cache.clustersize[0] < C.clustersize[0]):
                            cache.clustersize = [C.clustersize[0]]
                        self.CPA.counter += 1
                    elif self.CPA.counter > self.CPA.data.m - 1:
                        self.CachePointsCC[0] = True
                        self.CachePointsCC[1] = self.CPA.m
                        self.CachePointsTW[0] = True
                        self.CachePointsTW[1] = self.CPA.m
                        self.CPA_state = CPA_State.DetectCluster
                        self.CPA.exec = True


    def end_of_cycle(self):
        self.CPA.exec = False
        self.VisibleClustersCA[0] = False


def robot_index(namespace):
    """0 for /T3A, 1 for /T3B, ...; 0 when the namespace does not end in a letter."""
    tail = namespace.rstrip('/')[-1:] if namespace else ''
    return ord(tail.upper()) - ord('A') if tail.isalpha() else 0


class CachePointAssignmentNode(Node):
    def __init__(self):
        super().__init__('cache_point_assignment')
        seed = self.declare_parameter('random_seed', 0).value
        self.machine = CachePointAssignmentMachine(
            self.get_logger(), initial=initial_caches(seed, robot_index(self.get_namespace())))
        self.create_subscription(VisibleClustersCC, 'VisibleClustersCC', self.VCCC_callback, 10)
        self.create_subscription(Int32, 'CurrentTypeCA',
                                 lambda msg: setattr(self.machine, 'CurrentTypeCA', [True, msg.data]), 10)
        self.CachePointsCC_pub = self.create_publisher(CachePointsCC, 'CachePointsCC', 10)
        self.CachePointsTW_pub = self.create_publisher(CachePointsCC, 'CachePointsTW', 10)
        self.create_timer(self.machine.timer_period, self.tick)

    def VCCC_callback(self, msg):
        if self.machine.VisibleClustersCA[0] == False and not msg.n == 0:
            self.machine.VisibleClustersCA = [True, msg]

    def tick(self):
        self.machine.CachePointAssignment()
        self.events_out()
        self.machine.end_of_cycle()

    def events_out(self):
        machine = self.machine
        if machine.CachePointsCC[0] or machine.CachePointsTW[0]:
            message = CachePointsCC(clusterdata=copy.deepcopy(machine.CPA.m))
            self.CachePointsCC_pub.publish(message)
            self.CachePointsTW_pub.publish(message)
            machine.CachePointsCC[0] = machine.CachePointsTW[0] = False


def main(args=None):
    rclpy.init(args=args)
    node = CachePointAssignmentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
