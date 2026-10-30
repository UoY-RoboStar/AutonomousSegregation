"""ObstacleAvoidance state machine: steers around the closest lidar return, else forwards NOAMove."""
from enum import Enum

import rclpy
from rclpy.node import Node
import math

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty


class OA_State(Enum):
    OAEnabled = 0
    OADisabled = 1


class OA():
            T = 0
            av = 0.7
            lv = 0.8
            pi = 3.14159
            closest_angle = 0
            closest_distance = 0
            min_range = 0.2
            max_range = 1
            exec = False


class ObstacleAvoidanceMachine:
    def __init__(self, logger, timer_period=0.1):
        self.logger = logger
        self.timer_period = timer_period
        self.OA_state = OA_State.OAEnabled
        self.OA = OA()
        # Events in
        self.EnableOA = True
        self.DisableOA = False
        self.NOAMove = [False, (0.0, 0.0)]
        # Events out
        self.OAMove_temp = [False, (0.0, 0.0)]

    def get_logger(self):
        return self.logger

    def update_scan(self, msg: LaserScan):
        #reset range of max of msg
        # self.min_range = msg.range_min
        # self.max_range = msg.range_max
        self.OA.closest_distance = msg.range_max

        # Identify closest obstacle within the angle range of -120 to 120 degrees
        for angle_index, scan_range in enumerate(msg.ranges):
            # Convert the index to the corresponding angle in degrees
            angle = angle_index if angle_index <= 180 else angle_index - 360
            # Check if the angle is within the -110 to 110 degree range
            if -150 <= angle <= 150:
                # Check if the range is greater than the desired minimum (to avoid e.g. robot components) and less than the current closest distance and the maximum range
                if scan_range >= self.OA.min_range and scan_range < self.OA.closest_distance and scan_range < self.OA.max_range:
                    self.OA.closest_distance = scan_range
                    self.OA.closest_angle = angle

    def ObstacleAvoidance(self):
        while self.OA.exec == False:
            match self.OA_state:
                case OA_State.OAEnabled:
                    if self.DisableOA == True:
                        self.OA_state = OA_State.OADisabled
                        self.OA.exec = True
                    else:
                        # If obstacle is nearby
                        if self.OA.closest_distance >= self.OA.min_range and self.OA.closest_distance < self.OA.max_range and  abs(self.OA.closest_angle) <= 90: 
                            #turn away from the obstacle
                            if self.OA.closest_angle > 0:
                                av = (-100 + self.OA.closest_angle) / 180 * math.pi
                            else:
                                av = (100 + self.OA.closest_angle) / 180 * math.pi
                            current_speed = self.NOAMove[1][1]
                            lv = current_speed / 2 if self.OA.closest_distance > 0.4 else 0.0
                            if abs(self.OA.closest_angle) < 30:
                            # Reverse if directly ahead and very close, or only turn
                                lv = -0.4 if self.OA.closest_distance < 0.4 else 0.0
                        
                        else:
                            # If no obstacle is nearby, continue with the current movement
                            lv = self.NOAMove[1][1]
                            av = self.NOAMove[1][0]  
                        
                        #Store the moves in temp variables that will be published once all of the execs are called. 
                        self.OAMove_temp[0] = True
                        self.OAMove_temp[1] = (av, lv)
                        self.OA.exec = True
                case OA_State.OADisabled:
                    if self.EnableOA == True:
                        self.OA_state = OA_State.OAEnabled
                        self.OA.exec = True
                    elif self.EnableOA == False:
                        self.OA.exec = True

    def end_of_cycle(self):
        self.OA.T += self.timer_period
        self.OA.exec = False
        self.EnableOA = True
        self.DisableOA = False
        self.NOAMove[0] = False


class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')
        self.machine = ObstacleAvoidanceMachine(self.get_logger())
        self.create_subscription(LaserScan, 'scan', self.machine.update_scan, 10)
        self.create_subscription(Empty, 'EnableOA', lambda msg: setattr(self.machine, 'EnableOA', True), 10)
        self.create_subscription(Empty, 'DisableOA', lambda msg: setattr(self.machine, 'DisableOA', True), 10)
        self.create_subscription(Twist, 'NOAMove', lambda msg: setattr(
            self.machine, 'NOAMove', [True, (msg.angular.z, msg.linear.x)]), 10)
        self.OAMove_pub = self.create_publisher(Twist, 'OAMove', 10)
        self.create_timer(self.machine.timer_period, self.tick)

    def tick(self):
        self.machine.ObstacleAvoidance()
        self.events_out()
        self.machine.end_of_cycle()

    def events_out(self):
        if self.machine.OAMove_temp[0]:
            twist = Twist()
            twist.angular.z = float(self.machine.OAMove_temp[1][0])
            twist.linear.x = float(self.machine.OAMove_temp[1][1])
            self.OAMove_pub.publish(twist)
            self.machine.OAMove_temp = [False, (0.0, 0.0)]


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
