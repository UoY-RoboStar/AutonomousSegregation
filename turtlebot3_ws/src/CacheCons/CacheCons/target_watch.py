"""TargetWatch state machine: picks the object to collect from the target cluster."""
from enum import Enum

import rclpy
from rclpy.node import Node
import math

from geometry_msgs.msg import Point
from interfaces.msg import CachePointsCC, InvalidTarget, PuckData, TargetPuck, VisibleClustersCC
from std_msgs.msg import Empty, Int32

from .helpers import zero_cluster


class TW_State(Enum):
    WaitingForTarget = 0
    DetectingTarget = 1
    CalculateTargetObject = 2
    ValidateTargetObject = 3
    PublishTargetObject = 4
    WaitForDisableTargetWatch = 5


class TW():
    exec = False
    targetObjectxy = (0.0,0.0)
    closestTargetObject = PuckData()
    closestTargetObjectPosition = (0.0,0.0)
    counter = 0
    data = VisibleClustersCC()
    ObjectThreshold = 0.25
    done = False
    validObject = True
    targetType = 0
    m = [zero_cluster(), zero_cluster(), zero_cluster()]
    m[0].clustertype = [1]
    m[1].clustertype = [2]
    m[2].clustertype = [3]
    m[0].clustersize = [0]
    m[1].clustersize = [0]
    m[2].clustersize = [0]


class TargetWatchMachine:
    def __init__(self, logger, timer_period=0.1):
        self.logger = logger
        self.timer_period = timer_period
        self.TW_state = TW_State.WaitingForTarget
        self.TW = TW()
        # Events in
        self.VisibleClustersTW = [False, VisibleClustersCC()]
        self.EnableTargetWatch = [False, (0.0, 0.0)]
        self.DisableTargetWatch = False
        self.CurrentTypeTW = [False, 0]
        self.CachePointsTW = [False, [zero_cluster(), zero_cluster(), zero_cluster()]]
        # Events out
        self.TargetObject_temp = [False, (0.0, 0.0)]
        self.InvalidTarget_temp = False

    def get_logger(self):
        return self.logger

    def distance(self, x1, x2):
        return abs(math.sqrt((x1[0] - x2[0])**2 + (x1[1] - x2[1])**2))

    def TargetWatch(self):
        while self.TW.exec == False:
            match self.TW_state:
                case TW_State.WaitingForTarget:
                    if self.VisibleClustersTW[0] == True:
                        self.TW.data = self.VisibleClustersTW[1]
                    if self.EnableTargetWatch[0] == False:
                        self.TW_state = TW_State.WaitingForTarget
                        self.TW.exec = True

                    elif self.EnableTargetWatch[0] == True and self.CurrentTypeTW[0] == True:
                        self.TW.targetObjectxy = self.EnableTargetWatch[1]
                        self.TW.targetType = self.CurrentTypeTW[1]
                        self.TW.counter = 0
                        self.TW.closestTargetObject = self.TW.data.puckdata[0]
                        self.TW_state = TW_State.DetectingTarget
                        #self.TW.exec = True
                    elif self.EnableTargetWatch[0] == True and self.VisibleClustersTW[0] == False:
                        self.InvalidTarget_temp = True
                        self.TW_state = TW_State.WaitForDisableTargetWatch
                        self.TW.exec = True
                case TW_State.DetectingTarget | TW_State.CalculateTargetObject | TW_State.ValidateTargetObject | TW_State.PublishTargetObject | TW_State.WaitForDisableTargetWatch:
                    if self.VisibleClustersTW[0] == False:
                        self.TW_state = TW_State.WaitForDisableTargetWatch
                        self.InvalidTarget_temp = True
                        self.TW.exec = True
                    if self.DisableTargetWatch == True:
                        self.TW_state = TW_State.WaitingForTarget
                        self.TW.exec = True
                    match self.TW_state:
                        case TW_State.DetectingTarget:
                            self.TW_state = TW_State.CalculateTargetObject
                        case TW_State.CalculateTargetObject:
                            if self.distance(self.TW.data.puckdata[self.TW.counter].position, self.TW.targetObjectxy) <= self.distance(self.TW.closestTargetObject.position, self.TW.targetObjectxy):
                                self.get_logger().info("here1")
                                self.TW.closestTargetObject = self.TW.data.puckdata[self.TW.counter]
                            else:
                                self.get_logger().info("did not compute")
                            if self.TW.counter < self.TW.data.n - 1:
                                self.TW.counter += 1
                            elif self.TW.counter == self.TW.data.n - 1 and self.CachePointsTW[0] == True:
                                self.TW.validObject = True
                                self.TW_state = TW_State.ValidateTargetObject
                                self.TW.m = self.CachePointsTW[1]
                                self.TW.counter = 0
                            else:
                                self.TW.exec = True
                                
                        case TW_State.ValidateTargetObject:
                            if self.distance(self.TW.closestTargetObject.position, self.TW.targetObjectxy) > self.TW.ObjectThreshold:
                                self.TW.validObject = False
                            if self.TW.validObject == True:
                                self.TW_state = TW_State.PublishTargetObject
                            elif self.TW.validObject == False:
                                self.InvalidTarget_temp = True
                                self.get_logger().info("Failed because validObject is False")
                                #time.sleep(10)
                                self.TW_state = TW_State.WaitForDisableTargetWatch
                        case TW_State.PublishTargetObject:
                            self.TW.closestTargetObjectPosition = self.TW.closestTargetObject.position
                            self.TW.counter = 0
                            self.TargetObject_temp[0] = True
                            self.TargetObject_temp[1] = self.TW.closestTargetObjectPosition
                            self.TW_state = TW_State.CalculateTargetObject
                            self.TW.exec = True

                        case TW_State.WaitForDisableTargetWatch:
                            self.InvalidTarget_temp = True
                            self.TW.exec = True
                              

    def end_of_cycle(self):
        self.TW.exec = False
        self.VisibleClustersTW[0] = False
        self.EnableTargetWatch[0] = False
        self.DisableTargetWatch = False


class TargetWatchNode(Node):
    def __init__(self):
        super().__init__('target_watch')
        self.machine = TargetWatchMachine(self.get_logger())
        self.create_subscription(VisibleClustersCC, 'VisibleClustersCC', self.VCCC_callback, 10)
        self.create_subscription(Point, 'EnableTargetWatch', lambda msg: setattr(
            self.machine, 'EnableTargetWatch', [True, (msg.x, msg.y)]), 10)
        self.create_subscription(Empty, 'DisableTargetWatch',
                                 lambda msg: setattr(self.machine, 'DisableTargetWatch', True), 10)
        self.create_subscription(Int32, 'CurrentTypeTW',
                                 lambda msg: setattr(self.machine, 'CurrentTypeTW', [True, msg.data]), 10)
        self.create_subscription(CachePointsCC, 'CachePointsTW', self.CachePointsTW_callback, 10)
        self.TargetPuck_pub = self.create_publisher(TargetPuck, 'TargetPuck', 10)
        self.InvalidTarget_pub = self.create_publisher(InvalidTarget, 'InvalidTarget', 10)
        self.create_timer(self.machine.timer_period, self.tick)

    def VCCC_callback(self, msg):
        if self.machine.VisibleClustersTW[0] == False and not msg.n == 0:
            self.machine.VisibleClustersTW = [True, msg]

    def CachePointsTW_callback(self, msg):
        # Replace in place: TW.m keeps pointing at this list, as it shared CPA.m in one node.
        self.machine.CachePointsTW[1][:] = list(msg.clusterdata)
        self.machine.CachePointsTW[0] = True

    def tick(self):
        self.machine.TargetWatch()
        self.events_out()
        self.machine.end_of_cycle()

    def events_out(self):
        machine = self.machine
        if machine.TargetObject_temp[0]:
            self.TargetPuck_pub.publish(TargetPuck(targetpuck=[float(v) for v in machine.TargetObject_temp[1]]))
            machine.TargetObject_temp = [False, (0.0, 0.0)]
        if machine.InvalidTarget_temp:
            self.InvalidTarget_pub.publish(InvalidTarget(invalidtarget=True))
            machine.InvalidTarget_temp = False


def main(args=None):
    rclpy.init(args=args)
    node = TargetWatchNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
