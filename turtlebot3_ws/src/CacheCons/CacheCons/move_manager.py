"""MoveManager state machine: arbitrates OAMove, RWMove and CCMove into cmd_vel."""
from enum import Enum

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from .helpers import zero_twist


class MM_State(Enum):
    MoveHandler = 0


class MM():
        T = 0
        exec = False


class MoveManagerMachine:
    def __init__(self, logger, Move, timer_period=0.1):
        self.logger = logger
        self.timer_period = timer_period
        self.MM_state = MM_State.MoveHandler
        self.MM = MM()
        # Events in
        self.OAMove = [False, (0.0, 0.0)]
        self.RWMove = [False, (0.0, 0.0)]
        self.CCMove = [False, (0.0, 0.0)]
        # Events out
        self.NOAMove_temp = [False, (0.0, 0.0)]
        # Operations out: the platform's cmd_vel publisher.
        self.Move = Move

    def get_logger(self):
        return self.logger

    def MoveManager(self):
        while self.MM.exec == False:
            match self.MM_state:
                case MM_State.MoveHandler:
                    #condition
                    #self.get_logger().info(f"OA {self.OAMove[0]} RW {self.RWMove[0]} CC {self.CCMove[0]}")
                    if self.OAMove[0] == True and self.RWMove[0] == True and self.CCMove[0] == False:
                        self.MM_state = MM_State.MoveHandler
                        av = self.OAMove[1][0]
                        lv = self.OAMove[1][1]
                        msg = zero_twist()
                        msg.linear.x = lv
                        msg.angular.z = av
                        #self.get_logger().info(f"lv {lv} av {av}")
                        self.Move.publish(msg)
                        av = self.RWMove[1][0]
                        lv = self.RWMove[1][1]
                        self.NOAMove_temp[0] = True
                        self.NOAMove_temp[1] = (av,lv)       
                    # elif self.OAMove[0] == True and self.CCMove[0] == True:
                    #     self.MM_state = MM_State.MoveHandler
                    #     av = self.OAMove[1][0]
                    #     lv = self.OAMove[1][1]
                    #     msg = zero_twist()
                    #     msg.linear.x = lv
                    #     msg.angular.z = av
                    #     #self.get_logger().info(f"lv {lv} av {av}")
                    #     self.Move.publish(msg)
                    #     av = self.CCMove[1][0]
                    #     lv = self.CCMove[1][1]
                    #     self.NOAMove_temp[0] = True
                    #     self.NOAMove_temp[1] = (av,lv)
                    elif  self.OAMove[0] == False and self.RWMove[0] == True and self.CCMove[0] == False:
                        self.MM_state = MM_State.MoveHandler
                        av = self.RWMove[1][0]
                        lv = self.RWMove[1][1]  
                        msg = zero_twist()
                        msg.linear.x = lv
                        msg.angular.z = av
                        #self.get_logger().info(f"lv {lv} av {av}")
                        self.Move.publish(msg)
                        self.NOAMove_temp[0] = True
                        self.NOAMove_temp[1] = (av,lv)
                    elif self.CCMove[0] == True and self.OAMove[0] == False:
                        self.MM_state = MM_State.MoveHandler
                        av = self.CCMove[1][0]
                        lv = self.CCMove[1][1]
                        msg = zero_twist()
                        msg.linear.x = lv
                        msg.angular.z = av
                        self.Move.publish(msg)
                        self.NOAMove_temp[0] = True
                        self.NOAMove_temp[1] = (av,lv)
                    elif self.OAMove[0] == True and self.RWMove[0] == False and self.CCMove[0] == True:
                        self.MM_state = MM_State.MoveHandler
                        av = self.OAMove[1][0]
                        lv = self.OAMove[1][1]
                        msg = zero_twist()
                        msg.linear.x = lv
                        msg.angular.z = av
                        self.Move.publish(msg)
                        av = self.CCMove[1][0]
                        lv = self.CCMove[1][1]
                        self.NOAMove_temp[0] = True
                        self.NOAMove_temp[1] = (av,lv)
                    # else:
                    #     self.get_logger().info("No moves")
                    self.MM.exec = True

    def end_of_cycle(self):
        self.MM.T += self.timer_period
        self.MM.exec = False
        self.OAMove[0] = False
        self.RWMove[0] = False
        self.CCMove[0] = False


class MoveManagerNode(Node):
    def __init__(self):
        super().__init__('move_manager')
        self.machine = MoveManagerMachine(self.get_logger(), self.create_publisher(Twist, 'cmd_vel', 10))
        for name in ('OAMove', 'RWMove', 'CCMove'):
            self.create_subscription(Twist, name, lambda msg, name=name: setattr(
                self.machine, name, [True, (msg.angular.z, msg.linear.x)]), 10)
        self.NOAMove_pub = self.create_publisher(Twist, 'NOAMove', 10)
        self.create_timer(self.machine.timer_period, self.tick)

    def tick(self):
        self.machine.MoveManager()
        self.events_out()
        self.machine.end_of_cycle()

    def events_out(self):
        if self.machine.NOAMove_temp[0]:
            twist = Twist()
            twist.angular.z = float(self.machine.NOAMove_temp[1][0])
            twist.linear.x = float(self.machine.NOAMove_temp[1][1])
            self.NOAMove_pub.publish(twist)
            self.machine.NOAMove_temp = [False, (0.0, 0.0)]


def main(args=None):
    rclpy.init(args=args)
    node = MoveManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
