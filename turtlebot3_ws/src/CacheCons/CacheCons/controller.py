"""The CacheConsC controller: all six state machines in one node under one 0.1 s cycle,
with the events wired as in the RoboSim model's connection list."""
import rclpy
from rclpy.node import Node
from simulation_interfaces.msg import StateMsg
from geometry_msgs.msg import Twist
from interfaces.msg import PoseO, VisibleClustersCC
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

from .cache_cons_s import CacheConsSMachine
from .cache_point_assignment import CachePointAssignmentMachine, initial_caches, robot_index
from .move_manager import MoveManagerMachine
from .obstacle_avoidance import ObstacleAvoidanceMachine
from .random_walk import RandomWalkMachine
from .target_watch import TargetWatchMachine


class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller')
        logger = self.get_logger()
        self.cc = CacheConsSMachine(logger, self.create_publisher(Bool, 'DepositPuck', 10),
                                    self.create_publisher(StateMsg, 'StateMsg', 1))
        self.rw = RandomWalkMachine(logger)
        self.tw = TargetWatchMachine(logger)
        seed = self.declare_parameter('random_seed', 0).value
        self.cpa = CachePointAssignmentMachine(
            logger, initial=initial_caches(seed, robot_index(self.get_namespace())))
        self.oa = ObstacleAvoidanceMachine(logger)
        self.mm = MoveManagerMachine(logger, self.create_publisher(Twist, 'cmd_vel', 10))
        # Platform events in
        self.create_subscription(VisibleClustersCC, 'VisibleClustersCC', self.VCCC_callback, 10)
        self.create_subscription(PoseO, 'PoseO', lambda msg: setattr(self.cc, 'Coord_O', [True, msg.poseo]), 10)
        self.create_subscription(Bool, 'PuckCarried', self.cc.ObjectCarried_callback, 10)
        self.create_subscription(LaserScan, 'scan', self.oa.update_scan, 10)
        self.create_timer(self.cc.timer_period, self.cycle)

    def VCCC_callback(self, msg):
        if self.cc.VisibleClustersCC[0] == False and not msg.n == 0:
            self.cc.VisibleClustersCC = [True, msg]
            self.cpa.VisibleClustersCA = [True, msg]
            self.tw.VisibleClustersTW = [True, msg]

    def cycle(self):
        self.events_in()
        self.oa.ObstacleAvoidance()
        self.mm.MoveManager()
        self.rw.RandomWalk()
        self.cpa.CachePointAssignment()
        self.tw.TargetWatch()
        self.cc.CacheConsS()
        for machine in (self.oa, self.mm, self.rw, self.cpa, self.tw, self.cc):
            machine.end_of_cycle()

    def events_in(self):
        """Deliver last cycle's events along the model's connections."""
        cc, rw, tw, cpa, oa, mm = self.cc, self.rw, self.tw, self.cpa, self.oa, self.mm
        if tw.InvalidTarget_temp:
            cc.InvalidTarget, tw.InvalidTarget_temp = True, False
        if tw.TargetObject_temp[0]:
            cc.TargetObject, tw.TargetObject_temp = tw.TargetObject_temp, [False, (0.0, 0.0)]
        if cc.EnableTargetWatch_temp[0]:
            tw.EnableTargetWatch, cc.EnableTargetWatch_temp = cc.EnableTargetWatch_temp, [False, (0.0, 0.0)]
        if cc.DisableTargetWatch_temp:
            tw.DisableTargetWatch, cc.DisableTargetWatch_temp = True, False
        if cc.EnableClusterWatch_temp:
            rw.EnableClusterWatch, cc.EnableClusterWatch_temp = True, False
        if cc.DisableClusterWatch_temp:
            rw.DisableClusterWatch, cc.DisableClusterWatch_temp = True, False
        if cc.EnableOA_temp:
            oa.EnableOA, cc.EnableOA_temp = True, False
        if cc.DisableOA_temp:
            oa.DisableOA, cc.DisableOA_temp = True, False
        if cc.CurrentTypeCA[0]:
            cpa.CurrentTypeCA, cc.CurrentTypeCA[0] = [True, cc.CurrentTypeCA[1]], False
        if cc.CurrentTypeTW[0]:
            tw.CurrentTypeTW, cc.CurrentTypeTW[0] = [True, cc.CurrentTypeTW[1]], False
        # The cache points are shared by reference, as CPA.m was within one node.
        if cpa.CachePointsCC[0]:
            cc.CachePointsCC, cpa.CachePointsCC[0] = [True, cpa.CPA.m], False
        if cpa.CachePointsTW[0]:
            tw.CachePointsTW, cpa.CachePointsTW[0] = [True, cpa.CPA.m], False
        if oa.OAMove_temp[0]:
            mm.OAMove, oa.OAMove_temp = oa.OAMove_temp, [False, (0.0, 0.0)]
        if rw.RWMove_temp[0]:
            mm.RWMove, rw.RWMove_temp = rw.RWMove_temp, [False, (0.0, 0.0)]
        if cc.CCMove_temp[0]:
            mm.CCMove, cc.CCMove_temp = cc.CCMove_temp, [False, (0.0, 0.0)]
        if mm.NOAMove_temp[0]:
            oa.NOAMove, mm.NOAMove_temp = mm.NOAMove_temp, [False, (0.0, 0.0)]


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
