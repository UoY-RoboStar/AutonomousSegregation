"""CacheConsS state machine: the cache consensus behaviour (scan, target, home, deposit, exile)."""
from enum import Enum

import rclpy
from rclpy.node import Node
import copy
import math
import time

import numpy as np
from simulation_interfaces.msg import StateMsg
from geometry_msgs.msg import Point, Twist
from interfaces.msg import CachePointsCC, InvalidTarget, PoseO, TargetPuck, VisibleClustersCC
from std_msgs.msg import Bool, Empty, Int32

from .helpers import randomcoef, zero_cluster


class CC_State(Enum):
    PU_SCAN = 0
    WaitForCluster = 1
    ClusterSeen = 2
    CalcSmallestVisibleCluster = 3
    CalculateProb = 4
    ChooseTargetPosition = 5
    CalcLeftObject = 6
    CalcRightObject = 7
    ChooseLeftObject = 8
    ChooseRightObject = 9
    PU_TARGET = 10
    MoveToTarget = 11
    TurnToTarget = 12
    LinearMoveToTarget = 13
    HOMING = 14
    TurnToHome = 15
    LinearMoveToHome = 16
    DE_PUSH = 17
    DE_BACKUP = 18
    EXILE = 19
    EXILE_Turn = 20
    EXILE_LinearMove = 21


class CC():
    #constants
    lv = 0.25
    av = 1.0
    pi = 3.14159
    randcoef = 100
    k1 = 1
    timeout = 6.0
    timeout_PUSH = 0.5
    timeout_BACKUP = 5.0
    timeout_EXILE = 5.0
    PUSH_LV = 0.2
    HOME_LV = 0.8
    LINEAR_HOME = 0.8
    TARGET_AV = 0.3
    ANGLE_DIFF = 0.1
    ANGLE_DIFF_TOLERANCE = 0.01
    angleLB = -0.02
    angleUB = 0.02
    #variables
    exec = False
    done = False
    T = 0
    distance = 0.0
    angle = 10
    prob = 0.0
    homeangle = 0.0
    counter = 0
    data = []
    SmallestVisibleCluster = zero_cluster()
    m = []
    leftObject = 0
    leftObjectx = 0.0
    leftObjecty = 0.0
    rightObject = 0
    rightObjectx = 0.0
    rightObjecty = 0.0
    targetObjectID = 0
    targetObjectx = 0.0
    targetObjecty = 0.0
    targetPosition = [False, (0.0,0.0)]
    targetObject = [False, (0.0,0.0)]
    ct = 0


class CacheConsSMachine:
    def __init__(self, logger, Deposit, StatePub, timer_period=0.1):
        self.logger = logger
        self.timer_period = timer_period
        self.CC_state = CC_State.PU_SCAN
        self.CC = CC()
        # Events in
        self.VisibleClustersCC = [False, VisibleClustersCC()]
        self.Coord_O = [False, PoseO()]
        self.ObjectCarried = False
        self.CachePointsCC = [False, [zero_cluster(), zero_cluster(), zero_cluster()]]
        self.TargetObject = [False, (0.0, 0.0)]
        self.InvalidTarget = False
        # Events out
        self.CCMove_temp = [False, (0.0, 0.0)]
        self.EnableClusterWatch_temp = False
        self.DisableClusterWatch_temp = False
        self.EnableOA_temp = False
        self.DisableOA_temp = False
        self.EnableTargetWatch_temp = [False, (0.0, 0.0)]
        self.DisableTargetWatch_temp = False
        self.CurrentTypeCA = [False, 0]
        self.CurrentTypeTW = [False, 0]
        # Operations out: the platform's DepositPuck and the viewer's StateMsg publishers.
        self.Deposit = Deposit
        self.StatePub = StatePub

    def get_logger(self):
        return self.logger

    def ObjectCarried_callback(self, msg):
        if msg.data == True:
            self.ObjectCarried = True
        else:
            self.ObjectCarried = False


    # def is_done(self):
    #     return self.done

    def calculate_turn_angle(self,robot_pose, puck_position):
        """Calculates the angle a robot needs to turn to face a puck.

        Args:
            robot_pose (list/tuple): [x, y, theta] where theta is the robot's heading in radians.
            puck_position (list/tuple): [x, y] coordinates of the puck.

        Returns:
            float: Angle in radians that the robot needs to turn (positive = counter-clockwise).
        """
        robot_x, robot_y, robot_theta = robot_pose
        puck_x, puck_y = puck_position
        # Calculate target heading
        delta_x = puck_x - robot_x
        delta_y = puck_y - robot_y
        target_heading = np.arctan2(delta_y, delta_x) 

        # Calculate angle to turn (ensuring it's within -pi to pi)
        turn_angle = target_heading - robot_theta
        turn_angle = (turn_angle + np.pi) % (2 * np.pi) - np.pi  

        return turn_angle
    
    # 

    def distance(self, x1, x2):
        return abs(math.sqrt((x1[0] - x2[0])**2 + (x1[1] - x2[1])**2))

    def CacheConsS(self):
        while self.CC.exec == False:
            self.get_logger().info(f"state {self.CC_state}")
            match self.CC_state:
                case CC_State.PU_SCAN | CC_State.WaitForCluster |  CC_State.ClusterSeen | CC_State.CalcSmallestVisibleCluster | CC_State.CalculateProb | CC_State.ChooseTargetPosition | CC_State.CalcLeftObject | CC_State.CalcRightObject | CC_State.ChooseLeftObject | CC_State.ChooseRightObject:
                    self.StatePub.publish(StateMsg(statemsg = 0))
                    if self.ObjectCarried == True:
                        # Accidental pick-up: home to the cache point of the current type.
                        self.CC.m = self.CachePointsCC[1]
                        self.CC_state = CC_State.HOMING
                        self.CCMove_temp[0] = True
                        self.CCMove_temp[1] = (0.0,0.0)
                        self.DisableTargetWatch_temp = True
                        self.CC.exec = True
                    elif self.CC.done == True:
                       
                        # self.DisableOA_temp = True
                        # self.EnableOA_temp = False
                        self.CC.done = False                 

                        self.CC_state = CC_State.PU_TARGET
                        self.CurrentTypeCA[0] = True
                        self.CurrentTypeCA[1] = self.CC.ct
                        self.CurrentTypeTW[0] = True
                        self.CurrentTypeTW[1] = self.CC.ct
                        
                        self.EnableClusterWatch_temp = False
                        self.CCMove_temp[0] = True
                        self.CCMove_temp[1] = (0.0,0.0)
                        self.CC.T = 0
                        self.CC.angle = 10
                        self.CC.exec = True
                        self.EnableTargetWatch_temp = [True, self.CC.targetPosition[1]]
                        
                        
                    match self.CC_state:
                        case CC_State.PU_SCAN:
                            
                            self.EnableClusterWatch_temp = True
                            self.DisableClusterWatch_temp = False
                            self.EnableOA_temp = True
                            self.CC.counter = 0
                            self.CC_state = CC_State.WaitForCluster
                        case CC_State.WaitForCluster:
                            if self.VisibleClustersCC[0] == False:
                                self.CC_state = CC_State.WaitForCluster
                                self.CC.exec = True
                            elif self.VisibleClustersCC[0] == True:
                                #self.get_logger().info(f"True2 {self.VisibleClustersCC[1].n}")
                                self.CC.data  = self.VisibleClustersCC[1]
                                self.CC.counter = 0
                                self.CC.SmallestVisibleCluster.clustersize = [0] 
                                self.DisableClusterWatch_temp = True
                                #self.DisableOA_temp = True
                                self.CCMove_temp[0] = True
                                self.CCMove_temp[1] = (0.0,0.0)
                                self.CC_state = CC_State.ClusterSeen
                        case CC_State.ClusterSeen | CC_State.CalcSmallestVisibleCluster | CC_State.CalculateProb:
                            #self.get_logger().info("ClusterSeen")
                            if not (self.CC.randcoef <= self.CC.prob and self.CC.SmallestVisibleCluster.clustersize[0] > 0):
                                self.CCMove_temp[0] = True
                                self.CCMove_temp[1] = (0.0,0.0)
                                self.CC.ct = self.CC.SmallestVisibleCluster.clustertype[0]
                                match self.CC_state:
                                    case CC_State.ClusterSeen:
                                        self.CC_state = CC_State.CalcSmallestVisibleCluster
                                    case CC_State.CalcSmallestVisibleCluster:
                                        #self.get_logger().info(f"counter{self.CC.counter}")
                                        #self.get_logger().info(f"clustersize {self.CC.data.clusterdata[self.CC.counter]}")
                                        if self.CC.data.clusterdata[self.CC.counter].clustersize > self.CC.SmallestVisibleCluster.clustersize:
                                            self.CC.SmallestVisibleCluster = self.CC.data.clusterdata[self.CC.counter]
                                        if self.CC.counter < self.CC.data.n -1:
                                            self.CC.counter += 1
                                        if self.CC.counter == self.CC.data.n - 1:
                                            self.CC_state = CC_State.CalculateProb
                                    case CC_State.CalculateProb:
                                        self.CC.randcoef = randomcoef()
                                        self.CC.prob = (self.CC.k1 / (self.CC.k1 + self.CC.SmallestVisibleCluster.clustersize[0])) * (self.CC.k1 / (self.CC.k1 + self.CC.SmallestVisibleCluster.clustersize[0]))
                                        #self.get_logger().info(f"prob {self.CC.prob}")
                            else: 
                                self.CC_state = CC_State.ChooseTargetPosition
                                self.CC.counter = 0
                                #self.get_logger().info("ChooseTargetPosition")
                        case CC_State.ChooseTargetPosition | CC_State.CalcLeftObject | CC_State.CalcRightObject | CC_State.ChooseLeftObject | CC_State.ChooseRightObject:
                            #self.get_logger().info(f"state {self.CC_state}")
                            match self.CC_state:
                                case CC_State.ChooseTargetPosition:
                                    # Object ids are per observation; forget last cycle's.
                                    self.CC.leftObject = 0
                                    self.CC.rightObject = 0
                                    self.CC_state = CC_State.CalcLeftObject

                                case CC_State.CalcLeftObject:
                                    if self.CC.data.puckdata[self.CC.counter].clusterid == self.CC.SmallestVisibleCluster.clusterid and self.CC.data.puckdata[self.CC.counter].positionx < self.CC.leftObjectx:
                                        self.CC.leftObjectx = self.CC.data.puckdata[self.CC.counter].positionx
                                        self.CC.leftObjecty = self.CC.data.puckdata[self.CC.counter].positiony
                                        self.CC.leftObject = self.CC.data.puckdata[self.CC.counter].puckid[0]
                                    if self.CC.counter < self.CC.data.n - 1:
                                        self.CC.counter += 1
                                    if self.CC.counter == self.CC.data.n - 1:
                                        self.CC.counter = 0
                                        self.CC_state = CC_State.CalcRightObject
                                case CC_State.CalcRightObject:
                                    if self.CC.data.puckdata[self.CC.counter].clusterid == self.CC.SmallestVisibleCluster.clusterid and self.CC.data.puckdata[self.CC.counter].positionx < self.CC.rightObjectx:
                                        self.CC.rightObjectx = self.CC.data.puckdata[self.CC.counter].positionx
                                        self.CC.rightObjecty = self.CC.data.puckdata[self.CC.counter].positiony
                                        self.CC.rightObject = self.CC.data.puckdata[self.CC.counter].puckid[0]
                                    if self.CC.counter < self.CC.data.n - 1:
                                        self.CC.counter += 1
                                    if self.CC.data.puckdata[self.CC.leftObject -1].graphorder[0] < self.CC.data.puckdata[self.CC.rightObject -1].graphorder[0]:
                                        self.CC_state = CC_State.ChooseLeftObject
                                    elif self.CC.data.puckdata[self.CC.leftObject -1].graphorder[0] >= self.CC.data.puckdata[self.CC.rightObject -1].graphorder[0]:
                                        self.CC_state = CC_State.ChooseRightObject
                                case CC_State.ChooseLeftObject:
                                        self.CC.targetPosition = [True, (self.CC.leftObjectx, self.CC.leftObjecty)]
                                        self.CC.done = True
                                case CC_State.ChooseRightObject:
                                        self.CC.targetPosition = [True, (self.CC.rightObjectx, self.CC.rightObjecty)]
                                        self.CC.done = True
                                        

                case CC_State.PU_TARGET | CC_State.MoveToTarget | CC_State.TurnToTarget | CC_State.LinearMoveToTarget:
                    
                    self.StatePub.publish(StateMsg(statemsg = 1))
                    if self.ObjectCarried == True:
                        # Every type has a cache point from the start (initialCaches), so the
                        # object is always taken home.
                        self.CC.m = self.CachePointsCC[1]
                        self.CC_state = CC_State.HOMING
                        self.CCMove_temp[0] = True
                        self.CCMove_temp[1] = (0.0,0.0)
                        self.DisableTargetWatch_temp = True
                        self.CC.exec = True
                    match self.CC_state:
                        case CC_State.PU_TARGET:
                            
                            # self.CC.angle = self.angle_between(self.CC.targetPosition[1], self.Coord_O[1])
                            self.get_logger().info(f"Coord_O2 {self.Coord_O[1]}")
                            self.CC_state = CC_State.MoveToTarget
                        case CC_State.MoveToTarget | CC_State.TurnToTarget | CC_State.LinearMoveToTarget:
                            match self.CC_state:
                                case CC_State.MoveToTarget:
                                    self.CC_state = CC_State.TurnToTarget
                                case CC_State.TurnToTarget:
                                    self.DisableOA_temp = True
                                    if self.TargetObject[0] == False:
                                        self.CC.exec = True
                                    if self.CC.angle > 0:
                                        self.CCMove_temp[0] = True
                                        self.CCMove_temp[1] = (self.CC.TARGET_AV*self.CC.av, 0.0)
                                    else:
                                        self.CCMove_temp[0] = True
                                        self.CCMove_temp[1] = (-self.CC.TARGET_AV*self.CC.av, 0.0)
                                    if abs(self.CC.angle - 0.1) > 0.01 > 0.01 and self.Coord_O[0] == True:
                                        target = [0,0]
                                        target[0] = self.TargetObject[1][0]
                                        target[1] = self.TargetObject[1][1]
                                        self.CC.angle = self.calculate_turn_angle(self.Coord_O[1], target)
                                        self.CC.exec = True
                                    elif abs(self.CC.angle - 0.1) > 0.01 <= 0.01:
                                        self.DisableOA_temp = False
                                        self.EnableOA_temp = True
                                        self.CC.T = 0
                                        self.CCMove_temp[0] = True
                                        self.CCMove_temp[1] = (0.0, 0.0)
                                        self.CC_state = CC_State.LinearMoveToTarget
                                        self.CC.exec = True
                                    else:
                                        self.CC.exec = True
                                case CC_State.LinearMoveToTarget:
                                    self.get_logger().info(f"Coord_O3 {self.Coord_O[1]}")
                                    self.CCMove_temp[0] = True
                                    self.CCMove_temp[1] = (0.0, self.CC.lv)
                                    if self.InvalidTarget == True or self.CC.T >= self.CC.timeout:
                                        self.CC_state = CC_State.PU_SCAN
                                        self.DisableTargetWatch_temp = True
                                        self.EnableOA_temp = True
                                        self.CC.exec = True
                                    elif self.InvalidTarget == False and self.CC.T >= self.CC.timeout:
                                        Coord_O = copy.deepcopy(self.Coord_O)
                                        self.CC_state = CC_State.TurnToTarget
                                        target = [0,0]
                                        target[0] = self.TargetObject[1][0]
                                        target[1] = self.TargetObject[1][1]
                                        self.CC.angle = self.calculate_turn_angle(self.Coord_O[1], target)
                                    self.CC.exec = True

                case CC_State.HOMING | CC_State.TurnToHome | CC_State.LinearMoveToHome:
                    self.StatePub.publish(StateMsg(statemsg = 2))
                     #TODO discuss this running in parallel
                    
                    # self.get_logger().info(f"m {self.CC.m}") 
                    # self.get_logger().info(f"ct {self.CC.ct}") 
                    distance = self.distance(self.CC.m[self.CC.ct-1].centroid, (self.Coord_O[1][0], self.Coord_O[1][1]))
                    if distance > 0.2:
                        self.get_logger().info(f"ct {self.CC.ct}") 
                        self.get_logger().info(f"m {self.CC.m}") 
                    self.get_logger().info(f"Coord_O {self.Coord_O[1]}")
                    self.get_logger().info(f"distance {self.distance(self.CC.m[self.CC.ct-1].centroid, (self.Coord_O[1][0], self.Coord_O[1][1]))}")
                    if self.ObjectCarried == False:
                        self.CC_state = CC_State.PU_SCAN
                        self.CCMove_temp[0] = True
                        self.CCMove_temp[1] = (0.0,0.0)
                        self.CC.exec = True
                    elif self.distance(self.CC.m[self.CC.ct-1].centroid, (self.Coord_O[1][0], self.Coord_O[1][1])) < self.CC.LINEAR_HOME:
                        self.CC_state = CC_State.DE_PUSH
                        self.CCMove_temp[0] = True
                        self.CCMove_temp[1] = (0.0,0.0)
                        self.CC.T = 0
                        self.CC.exec = True
                    match self.CC_state:
                        case CC_State.HOMING:
                            self.CC_state = CC_State.TurnToHome
                            Coord_O = copy.deepcopy(self.Coord_O)
                            self.CC.angle = self.CC.angle = self.calculate_turn_angle((Coord_O[1][0], Coord_O[1][1], Coord_O[1][2]),(self.CC.m[self.CC.ct-1].centroid[0], self.CC.m[self.CC.ct-1].centroid[1]))
                        case CC_State.TurnToHome:
                            self.DisableOA_temp = True
                            if self.CC.angle > 0:
                                self.CCMove_temp[0] = True
                                self.CCMove_temp[1] = (self.CC.TARGET_AV*self.CC.av, 0.0)
                            else:
                                self.CCMove_temp[0] = True
                                self.CCMove_temp[1] = (-self.CC.TARGET_AV*self.CC.av, 0.0)
                            if abs(self.CC.angle - 0.1) > 0.01 and self.Coord_O[0]: 
                                Coord_O = copy.deepcopy(self.Coord_O)
                                self.CC.angle = self.calculate_turn_angle((Coord_O[1][0], Coord_O[1][1], Coord_O[1][2]),(self.CC.m[self.CC.ct-1].centroid[0], self.CC.m[self.CC.ct-1].centroid[1]))
                                self.CC.exec = True
                            elif abs(self.CC.angle - 0.1) <= 0.01 and self.Coord_O[0]: 
                                self.CCMove_temp[0] = True
                                self.CCMove_temp[1] = (0.0, 0.0)
                                self.CC.T = 0
                                self.DisableOA_temp = False
                                self.EnableOA_temp = True
                                self.CC_state = CC_State.LinearMoveToHome
                                self.CC.exec = True
                            else:
                                self.CC.exec = True
                        case CC_State.LinearMoveToHome:
                            if self.CC.T < self.CC.timeout:
                                self.CCMove_temp[0] = True
                                self.CCMove_temp[1] = (0.0, self.CC.HOME_LV*self.CC.lv)
                                self.CC.exec = True
                            else:
                                self.CCMove_temp[0] = True
                                self.CCMove_temp[1] = (0.0, 0.0)
                                Coord_O = copy.deepcopy(self.Coord_O)
                                self.CC.angle = self.calculate_turn_angle((Coord_O[1][0], Coord_O[1][1], Coord_O[1][2]),(self.CC.m[self.CC.ct-1].centroid[0], self.
                                CC.m[self.CC.ct-1].centroid[1]))
                                time.sleep(0.5)
                                self.CC_state = CC_State.TurnToHome
                                self.DisableOA_temp = True
                                self.CC.exec = True
                         

                case CC_State.DE_PUSH:
                    self.StatePub.publish(StateMsg(statemsg = 3))
                    if self.CC.T < self.CC.timeout_PUSH:
                        self.CCMove_temp[0] = True
                        self.CCMove_temp[1] = (0.0, self.CC.PUSH_LV*self.CC.lv)
                        self.CC.exec = True
                    else:
                        self.CCMove_temp[0] = True
                        self.CCMove_temp[1] = (0.0, 0.0)
                        msg = Bool()
                        msg.data = True
                        self.Deposit.publish(msg)
                        self.CC.T = 0
                        self.CC_state = CC_State.DE_BACKUP
                        self.CC.exec = True
                case CC_State.DE_BACKUP:
                    self.StatePub.publish(StateMsg(statemsg = 4))
                    if self.ObjectCarried == True:
                        msg = Bool()
                        msg.data = True
                        self.Deposit.publish(msg)
                        self.Deposit.publish(msg)
                        self.CC.exec = True
                    elif self.ObjectCarried == False and self.CC.T < self.CC.timeout_BACKUP:
                        self.CCMove_temp[0] = True
                        self.CCMove_temp[1] = (0.0, -self.CC.lv)
                        self.CC.exec = True
                    elif self.ObjectCarried == False and self.CC.T >= self.CC.timeout_BACKUP:
                        self.CCMove_temp[0] = True
                        self.CCMove_temp[1] = (0.0, 0.0)
                        self.CC.T = 0
                        self.CC_state = CC_State.EXILE
                        self.CC.exec = True
                case CC_State.EXILE | CC_State.EXILE_Turn| CC_State.EXILE_LinearMove:
                    self.StatePub.publish(StateMsg(statemsg = 5))
                    if self.CC.T >= self.CC.timeout_EXILE:
                        self.CCMove_temp[0] = True
                        self.CCMove_temp[1] = (0.0, 0.0)
                        self.CC_state = CC_State.PU_SCAN
                        self.CC.exec = True
                    match self.CC_state:
                        case CC_State.EXILE:
                            self.CC_state = CC_State.EXILE_Turn
                        case CC_State.EXILE_Turn:
                            if (self.CC.T*2) <= self.CC.pi/self.CC.av:
                                self.CCMove_temp[0] = True
                                self.CCMove_temp[1] = (self.CC.av, 0.0)
                                self.CC.exec = True
                            elif (self.CC.T*2) > self.CC.pi/self.CC.av:
                                self.CCMove_temp[0] = True
                                self.CCMove_temp[1] = (0.0, self.CC.lv)
                                self.CC_state = CC_State.EXILE_LinearMove
                                self.CC.exec = True
                        case CC_State.EXILE_LinearMove:
                            if self.CC.T < self.CC.timeout_EXILE:
                                self.CCMove_temp[0] = True
                                self.CCMove_temp[1] = (0.0, self.CC.lv)
                                self.CC.exec = True
                            
                                

            

    def end_of_cycle(self):
        self.CC.T += self.timer_period
        self.CC.exec = False
        self.VisibleClustersCC[0] = False
        self.InvalidTarget = False
        self.TargetObject[0] = False


class CacheConsSNode(Node):
    def __init__(self):
        super().__init__('cache_cons_s')
        self.machine = CacheConsSMachine(self.get_logger(), self.create_publisher(Bool, 'DepositPuck', 10),
                                         self.create_publisher(StateMsg, 'StateMsg', 1))
        self.create_subscription(VisibleClustersCC, 'VisibleClustersCC', self.VCCC_callback, 10)
        self.create_subscription(PoseO, 'PoseO', lambda msg: setattr(self.machine, 'Coord_O', [True, msg.poseo]), 10)
        self.create_subscription(Bool, 'PuckCarried', self.machine.ObjectCarried_callback, 10)
        self.create_subscription(CachePointsCC, 'CachePointsCC', self.CachePointsCC_callback, 10)
        self.create_subscription(TargetPuck, 'TargetPuck', lambda msg: setattr(
            self.machine, 'TargetObject', [True, tuple(msg.targetpuck)]), 10)
        self.create_subscription(InvalidTarget, 'InvalidTarget',
                                 lambda msg: setattr(self.machine, 'InvalidTarget', msg.invalidtarget), 10)
        self.CCMove_pub = self.create_publisher(Twist, 'CCMove', 10)
        self.EnableClusterWatch_pub = self.create_publisher(Empty, 'EnableClusterWatch', 10)
        self.DisableClusterWatch_pub = self.create_publisher(Empty, 'DisableClusterWatch', 10)
        self.EnableOA_pub = self.create_publisher(Empty, 'EnableOA', 10)
        self.DisableOA_pub = self.create_publisher(Empty, 'DisableOA', 10)
        self.EnableTargetWatch_pub = self.create_publisher(Point, 'EnableTargetWatch', 10)
        self.DisableTargetWatch_pub = self.create_publisher(Empty, 'DisableTargetWatch', 10)
        self.CurrentTypeCA_pub = self.create_publisher(Int32, 'CurrentTypeCA', 10)
        self.CurrentTypeTW_pub = self.create_publisher(Int32, 'CurrentTypeTW', 10)
        self.wait_for_machines = bool(self.declare_parameter('wait_for_machines', True).value)
        self.create_timer(self.machine.timer_period, self.tick)

    def VCCC_callback(self, msg):
        if self.machine.VisibleClustersCC[0] == False and not msg.n == 0:
            self.machine.VisibleClustersCC = [True, msg]

    def CachePointsCC_callback(self, msg):
        # Replace in place: CC.m keeps pointing at this list, as it shared CPA.m in one node.
        self.machine.CachePointsCC[1][:] = list(msg.clusterdata)
        self.machine.CachePointsCC[0] = True

    def machines_connected(self):
        """The other five state machines are subscribed, as when all six run in one node."""
        return all(publisher.get_subscription_count() > 0 for publisher in (
            self.CCMove_pub, self.EnableClusterWatch_pub, self.EnableOA_pub,
            self.EnableTargetWatch_pub, self.CurrentTypeCA_pub))

    def tick(self):
        if self.wait_for_machines and not self.machines_connected():
            return
        self.machine.CacheConsS()
        self.events_out()
        self.machine.end_of_cycle()

    def events_out(self):
        machine = self.machine
        if machine.CCMove_temp[0]:
            twist = Twist()
            twist.angular.z = float(machine.CCMove_temp[1][0])
            twist.linear.x = float(machine.CCMove_temp[1][1])
            self.CCMove_pub.publish(twist)
            machine.CCMove_temp = [False, (0.0, 0.0)]
        for name in ('EnableClusterWatch', 'DisableClusterWatch', 'EnableOA', 'DisableOA',
                     'DisableTargetWatch'):
            if getattr(machine, name + '_temp'):
                getattr(self, name + '_pub').publish(Empty())
                setattr(machine, name + '_temp', False)
        if machine.EnableTargetWatch_temp[0]:
            x, y = machine.EnableTargetWatch_temp[1]
            self.EnableTargetWatch_pub.publish(Point(x=float(x), y=float(y)))
            machine.EnableTargetWatch_temp = [False, (0.0, 0.0)]
        for name in ('CurrentTypeCA', 'CurrentTypeTW'):
            if getattr(machine, name)[0]:
                getattr(self, name + '_pub').publish(Int32(data=int(getattr(machine, name)[1])))
                getattr(machine, name)[0] = False


def main(args=None):
    rclpy.init(args=args)
    node = CacheConsSNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
