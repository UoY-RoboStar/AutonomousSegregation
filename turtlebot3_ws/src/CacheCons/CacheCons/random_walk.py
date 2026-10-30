"""RandomWalk state machine: wanders while cluster watching is enabled."""
from enum import Enum

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

from .helpers import randomcoef, random_sign


class RW_State(Enum):
    Wait = 0
    Wander = 1
    MoveForward = 2
    Turn = 3


class RW():
            T = 0
            av = 0.6
            lv = 0.7
            pi = 3.14159
            randcoef = 0.2
            exec = False
            sign = 1


class RandomWalkMachine:
    def __init__(self, logger, timer_period=0.1):
        self.logger = logger
        self.timer_period = timer_period
        self.RW_state = RW_State.Wait
        self.RW = RW()
        # Events in
        self.EnableClusterWatch = False
        self.DisableClusterWatch = False
        # Events out
        self.RWMove_temp = [False, (0.0, 0.0)]

    def get_logger(self):
        return self.logger

    def RandomWalk(self):
        while self.RW.exec == False:
            match self.RW_state:
                case RW_State.Wait:
                    if self.EnableClusterWatch == True:
                        self.RW_state = RW_State.MoveForward
                        self.RW.exec = True
                    else:
                        # self.RWMove_temp[0] = True
                        # self.RWMove_temp[1] = (0.0,0.0)
                        self.RW.exec = True
                case RW_State.Wander | RW_State.MoveForward | RW_State.Turn:
                    if self.DisableClusterWatch == True:
                        self.RWMove_temp[0] = True
                        self.RWMove_temp[1] = (0.0,0.0)
                        self.RW_state = RW_State.Wait
                        self.RW.exec = True
                    else:
                        match self.RW_state:
                            case RW_State.Turn:
                                #entry
                                av = self.RW.sign * self.RW.av
                                lv = 0.0
                                self.RWMove_temp[0] = True
                                self.RWMove_temp[1] = (av,lv)
                                #transitions
                                if self.RW.T < self.RW.randcoef:
                                    self.RW.exec = True
                                elif self.RW.T >= self.RW.randcoef:
                                    self.RWMove_temp[0] = True
                                    self.RWMove_temp[1] = (0.0,0.0)
                                    self.RW.T = 0
                                    self.RW_state = RW_State.MoveForward
                                    self.RW.randcoef = randomcoef()
                                    self.RW.sign = random_sign()
                                    self.RW.exec

                                self.RW.exec = True
                            case RW_State.MoveForward:
                                #entry
                                av = 0.0
                                lv = self.RW.lv
                                self.RWMove_temp[0] = True
                                self.RWMove_temp[1] = (av,lv)
                                #transitions
                                if self.RW.T < self.RW.randcoef:
                                    self.RW.exec = True
                                elif self.RW.T >= self.RW.randcoef:
                                    self.RWMove_temp[0] = True
                                    self.RWMove_temp[1] = (0.0,0.0)
                                    self.RW.T = 0
                                    self.RW_state = RW_State.Wander.Turn
                                    self.RW.randcoef = randomcoef()
                                    self.RW.exec = True

    def end_of_cycle(self):
        self.RW.T += self.timer_period
        self.RW.exec = False
        self.EnableClusterWatch = False
        self.DisableClusterWatch = False


class RandomWalkNode(Node):
    def __init__(self):
        super().__init__('random_walk')
        self.machine = RandomWalkMachine(self.get_logger())
        self.create_subscription(Empty, 'EnableClusterWatch',
                                 lambda msg: setattr(self.machine, 'EnableClusterWatch', True), 10)
        self.create_subscription(Empty, 'DisableClusterWatch',
                                 lambda msg: setattr(self.machine, 'DisableClusterWatch', True), 10)
        self.RWMove_pub = self.create_publisher(Twist, 'RWMove', 10)
        self.create_timer(self.machine.timer_period, self.tick)

    def tick(self):
        self.machine.RandomWalk()
        self.events_out()
        self.machine.end_of_cycle()

    def events_out(self):
        if self.machine.RWMove_temp[0]:
            twist = Twist()
            twist.angular.z = float(self.machine.RWMove_temp[1][0])
            twist.linear.x = float(self.machine.RWMove_temp[1][1])
            self.RWMove_pub.publish(twist)
            self.machine.RWMove_temp = [False, (0.0, 0.0)]


def main(args=None):
    rclpy.init(args=args)
    node = RandomWalkNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
