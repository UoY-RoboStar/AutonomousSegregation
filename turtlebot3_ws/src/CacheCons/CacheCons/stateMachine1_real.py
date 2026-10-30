# Import the necessary libraries
import os
import sys
import math
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
import rclpy  # Python library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
from sensor_msgs.msg import Image 
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String  
from std_msgs.msg import Bool
from std_msgs.msg import Float32  
from geometry_msgs.msg import Twist
import time
from rclpy.clock import ROSClock
import numpy as np
import copy
from interfaces.msg import PoseO, PuckCarried, VisibleClustersCC, PuckData, ClusterData
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from enum import Enum
from simulation_interfaces.msg import StateMsg

from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
import cv2  # OpenCV library
class State(Enum):
    WaitForCLuster = 0
    ClusterSeen = 1
    PU_TARGET = 2

class RW_State(Enum):
    Wait = 0
    Wander = 1
    MoveForward = 2
    Turn = 3

class MM_State(Enum):
    MoveHandler = 0

class OA_State(Enum):
    OAEnabled = 0
    OADisabled = 1

class CC_State(Enum):
    PU_SCAN = 0
    WaitForCluster = 1
    ClusterSeen = 2
    CalcSmallestVisibleCluster = 3
    CalculateProb = 4
    ChooseTargetPosition = 5
    CalcLeftPuck = 6
    CalcRightPuck = 7
    ChooseLeftPuck = 8
    ChooseRightPuck = 9
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

class CPA_State(Enum):
    DetectCluster = 0
    CalculateL = 1
    DecideNewCachePoint = 2
    UpdateCache = 3
    UpdateCurrentCache = 4

class TW_State(Enum):
    WaitingForTarget = 0
    DetectingTarget = 1
    CalculateTargetPuck = 2
    ValidateTargetPuck = 3
    PublishTargetPuck = 4
    WaitForDisableTargetWatch = 5

def randomcoef():
    return np.random.uniform(0, 4)

def random_sign():
    return np.random.choice([-1, 1])

def randomnat():
    return np.random.randint(0, 5)

def zero_VCCC(n):
    VCCC.n = n
    VCCC = VisibleClustersCC()
    puck = PuckData()
    puck.clusterid = [0]
    puck.graphorder = [1]
    for i in range(1,n+1):
        VCCC.puckdata.append(copy.deepcopy(puck))
    # for puck in VCCC.puckdata:
    #     puck.clusterid = 0
    return VCCC

def zero_cluster():
    cluster = ClusterData()
    cluster.clusterid = [0]
    cluster.clustersize = [0]
    cluster.clustertype = [0]
    cluster.centroid = (0.0,0.0)
    return cluster

def zero_twist():
    twist = Twist()
    twist.linear.x = 0.0
    twist.linear.y = 0.0
    twist.linear.z = 0.0

    twist.angular.x = 0.0
    twist.angular.y = 0.0
    twist.angular.z = 0.0

    return twist

avoiding = True 

# class ClusterData():
#     clusterID= 0
#     clusterSize = 0
#     clusterType = 0
#     centroid = (0.0,0.0)

class MM():
        T = 0
        exec = False
class OA():
            T = 0
            av = 0.7
            lv = 0.07
            pi = 3.14159
            closest_angle = 0
            closest_distance = 0
            min_range = 0.1
            max_range = 0.4
            exec = False

class RW():
            T = 0
            av = 0.6
            lv = 0.07
            pi = 3.14159
            randcoef = 0.2
            exec = False
            sign = 1

class CPA():
    exec = False
    j = 0
    counter = 0
    L = zero_cluster()
    data = VisibleClustersCC()
    threshold = 0.02
    m = [zero_cluster(), zero_cluster(), zero_cluster()]
    m[0].clustertype = [1]
    m[1].clustertype = [2]
    m[2].clustertype = [3]
    m[0].clustersize = [0]
    m[1].clustersize = [0]
    m[2].clustersize = [0]

class TW():
    exec = False
    targetPuckxy = (0.0,0.0)
    closestTargetPuck = PuckData()
    closestTargetPuckPosition = (0.0,0.0)
    counter = 0
    data = VisibleClustersCC()
    PuckThreshold = 0.25
    done = False
    validPuck = True
    targetType = 0
    m = [zero_cluster(), zero_cluster(), zero_cluster()]
    m[0].clustertype = [1]
    m[1].clustertype = [2]
    m[2].clustertype = [3]
    m[0].clustersize = [0]
    m[1].clustersize = [0]
    m[2].clustersize = [0]

class CC():
    #constants
    lv = 0.03
    av = 1.0
    pi = 3.14159
    randcoef = 100
    k1 = 1.0
    timeout = 6.0
    timeout_PUSH = 0.5
    timeout_BACKUP = 5.0
    timeout_EXILE = 5.0
    #variables
    exec = False
    done = False
    probDone = False
    T = 0
    distance = 0.0
    angle = 10
    prob = 0.0
    counter = 0
    data = []
    SmallestVisibleCluster = zero_cluster()
    m = []
    leftPuck = 0
    leftPuckx = 0.0
    leftPucky = 0.0
    rightPuck = 0
    rightPuckx = 0.0
    rightPucky = 0.0
    targetPuckID = 0
    targetPuckx = 0.0
    targetPucky = 0.0
    targetPosition = [False, (0.0,0.0)]
    targetPuck = [False, (0.0,0.0)]
    j = 0



class stateMachine1_real(Node):

    def __init__(self):
        super().__init__('stateMachine1_real')
        # Callback group
        self.cb_group_ = MutuallyExclusiveCallbackGroup()
        self.RW_state = RW_State.Wait
        self.MM_state = MM_State.MoveHandler
        self.OA_state = OA_State.OAEnabled
        self.CC_state = CC_State.PU_SCAN
        self.CPA_state = CPA_State.DetectCluster
        self.TW_state = TW_State.WaitingForTarget
        self.timer_period = 0.1 # 100 milliseconds = 10 Hz
        self.timer = self.create_timer(self.timer_period, self.eventsIn, self.cb_group_)
        self.timer = self.create_timer(self.timer_period, self.ObstacleAvoidance, self.cb_group_)
        self.timer = self.create_timer(self.timer_period, self.MoveManager, self.cb_group_)
        self.timer = self.create_timer(self.timer_period, self.RandomWalk, self.cb_group_)
        self.timer = self.create_timer(self.timer_period, self.CachePointAssignment, self.cb_group_)
        self.timer = self.create_timer(self.timer_period, self.TargetWatch, self.cb_group_)
        self.timer = self.create_timer(self.timer_period, self.CacheConsS, self.cb_group_) #TODO make sure to change this back to 0.1 once finished debugging
        self.timer = self.create_timer(self.timer_period, self.exec, self.cb_group_)
        #Image publisher for demos
        self.StatePub = self.create_publisher(StateMsg, 'StateMsg', 1) 
        #EventsIn
        ##CacheConsS
        self.VisibleClustersCC = [False,VisibleClustersCC()]
        self.VisibleClustersCC_sub = self.create_subscription(VisibleClustersCC, 'robot1/VisibleClustersCC', self.VCCC_callback, 10)
        self.Pose_O = [False,PoseO()]
        self.Pose_O_sub = self.create_subscription(PoseO, '/robot1/PoseO', self.PoseO_callback, 10)
        self.LaserScan = self.create_subscription(LaserScan, '/robot1/scan', self.update_scan, qos_profile_sensor_data)
        self.PuckCarried_sub = self.create_subscription(Bool, '/robot1/PuckCarried', self.PuckCarried_callback, 10)
        self.PuckCarried = False
        #EventsInternal
        ##To MoveManager
        self.RWMove = [False,(0.0,0.0)]
        self.OAMove = [False,(0.0,0.0)]
        self.CCMove = [False,(0.0,0.0)]
        ##To ObstacleAvoidance
        self.NOAMove= [False,(0.0,0.0)]
        self.EnableOA = True
        self.DisableOA = False
        self.EnableOA_temp = False
        self.DisableOA_temp = False
        ##To CacheConsS
        self.TargetPuck = [False,(0.0,0.0)]
        self.TargetPuck_temp = [False,(0.0,0.0)]
        self.InvalidTarget = False
        self.CachePointsCC = [False,[zero_cluster(),zero_cluster(),zero_cluster()]]
        ##To RandomWalk
        self.EnableClusterWatch = False
        self.DisableClusterWatch = False
        self.EnableClusterWatch_temp = False
        self.DisableClusterWatch_temp = False
        ##To CachePointAssignment
        self.CurrentTypeCA = [False,0]
        self.VisibleClustersCA = [False,VisibleClustersCC()]
        #To TargetWatch
        self.EnableTargetWatch = [False, (0.0,0.0)]
        self.EnableTargetWatch_temp = [False, (0.0,0.0)]
        self.DisableTargetWatch = False
        self.DisableTargetWatch_temp = False
        self.CurrentTypeTW = [False,0]
        self.TargetPuckTW = [False,(0.0,0.0)]
        self.CachePointsTW = [False,[zero_cluster(),zero_cluster(),zero_cluster()]]
        self.VisibleClustersTW = [False,VisibleClustersCC()]
        #Temp vars as we can't be sure that the events are updated synchronously in ROS2
        self.RWMove_temp = [False,(0.0,0.0)]
        self.OAMove_temp = [False,(0.0,0.0)]
        self.NOAMove_temp= [False,(0.0,0.0)]
        self.CCMove_temp = [False,(0.0,0.0)]
        self.InvalidTarget_temp = False


        #OperationsOut
        self.Move = self.create_publisher(Twist, '/robot1/cmd_vel', 10)
        self.Deposit = self.create_publisher(Bool, '/robot1/DepositPuck', 10)

        #Data
        #self.done = False



        #MM_ Variables
       
        self.MM = MM()

        #OA_ Variables
        
        self.OA = OA()

        #RW_ Variables
        

        self.RW = RW()
        self.CC = CC()
        self.CPA = CPA()
        self.TW = TW()

        self.OADone = False
        self.RWDone = False
        self.CPADone = False
        self.TWDone = False
        self.CCDone = False


    #set events for start of next cycle
    def eventsIn(self):
        if self.InvalidTarget_temp:
            self.InvalidTarget = self.InvalidTarget_temp
            self.InvalidTarget_temp = False
        if self.EnableTargetWatch_temp[0]:
            self.EnableTargetWatch = self.EnableTargetWatch_temp
            self.EnableTargetWatch_temp = [False, (0.0,0.0)]
        if self.DisableTargetWatch_temp:
            self.DisableTargetWatch = self.DisableTargetWatch_temp
            self.DisableTargetWatch_temp = False
        if self.EnableClusterWatch_temp:
            self.EnableClusterWatch = self.EnableClusterWatch_temp
            self.EnableClusterWatch_temp = False
        if self.DisableClusterWatch_temp:
            self.DisableClusterWatch = self.DisableClusterWatch_temp
            self.DisableClusterWatch_temp = False
        if self.EnableOA_temp:
            self.EnableOA = self.EnableOA_temp
            self.EnableOA_temp = False
        if self.DisableOA_temp:
            self.DisableOA = self.DisableOA_temp
            self.DisableOA_temp = False
        if self.TargetPuck_temp[0]:
            self.TargetPuck = self.TargetPuck_temp
            self.TargetPuck_temp = [False,(0.0,0.0)]

        if self.OAMove_temp[0]:
            self.OAMove = self.OAMove_temp
            self.OAMove_temp = [False,(0.0,0.0)]
        if self.RWMove_temp[0]:
            self.RWMove = self.RWMove_temp
            self.RWMove_temp = [False,(0.0,0.0)]
        if self.CCMove_temp[0]:
            self.CCMove = self.CCMove_temp
            self.CCMove_temp = [False,(0.0,0.0)]
        if self.NOAMove_temp[0]:
            self.NOAMove = self.NOAMove_temp
            self.NOAMove_temp = [False,(0.0,0.0)]

    def VCCC_callback(self, msg):
        if self.VisibleClustersCC[0] == False:
            if not msg.n == 0: #TODO need to fix ROS error which is publishing unwanted 0 clusters to remove this if statement
                self.VisibleClustersCC[1] = msg
                self.VisibleClustersCA[1] = msg
                self.VisibleClustersTW[1] = msg
                self.VisibleClustersCC[0] = True
                self.VisibleClustersCA[0] = True
                self.VisibleClustersTW[0] = True

    def PoseO_callback(self, msg):
        self.Pose_O[0] = True
        self.Pose_O[1] = msg.poseo

    def PuckCarried_callback(self, msg):
        if msg.data == True:
            self.PuckCarried = True
            print("PUCKCARRIED!")
        else:
            self.PuckCarried = False


    # def is_done(self):
    #     return self.done
    
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
            if -120 <= angle <= 120:
                # Check if the range is greater than the desired minimum (to avoid e.g. robot components) and less than the current closest distance and the maximum range
                if scan_range >= self.OA.min_range and scan_range < self.OA.closest_distance and scan_range < self.OA.max_range:
                    self.OA.closest_distance = scan_range
                    self.OA.closest_angle = angle


    def ObstacleAvoidance(self):
        while self.OA.exec == False:
            match self.OA_state:
                case OA_State.OAEnabled:
                    #self.get_logger().info(f"closest distance is {self.OA.closest_distance}")
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
        self.OADone = True
            

    def angle_between(self, target, current):
        # Calculate the angle between the target and the current position
        target_x, target_y = target
        current_x, current_y = current
        angle = math.atan2(target_y - current_y, target_x - current_x)
        # Calculate the difference between the current angle and the target angle
        #angle_diff = angle - self.Pose_O[1][2]
        # Normalize the angle difference to be in the range -pi to pi
        angle = (angle + math.pi) % (2 * math.pi) - math.pi
        return angle
    
    def angle_between_vectors(self, x, y):
        # Normalize the input vectors
        unit_x = x / np.linalg.norm(x)
        unit_y = y / np.linalg.norm(y)
        
        # Calculate the dot product
        dot_product = np.dot(unit_x, unit_y)
        
        # Calculate the cross product (z-component in 2D)
        cross_product = np.cross(unit_y, unit_x)
        
        # Calculate the angle using atan2 to get the correct angle in the range [-pi, pi]
        angle_rad = np.arctan2(cross_product, dot_product)
    
        return angle_rad
    
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
        puck_x = 0.45
        puck_y = 0.45
        # Calculate target heading
        delta_x = puck_x - robot_x
        delta_y = puck_y - robot_y
        target_heading = np.arctan2(delta_y, delta_x) 

        # Calculate angle to turn (ensuring it's within -pi to pi)
        turn_angle = target_heading - robot_theta
        turn_angle = (turn_angle + np.pi) % (2 * np.pi) - np.pi  

        return turn_angle
     
    
    def distance(self, x1, x2):
        return abs(math.sqrt((x1[0] - x2[0])**2 + (x1[1] - x2[1])**2))

    def RandomWalk(self):
        while self.RW.exec == False:
            self.get_logger().info(f"RW_State{self.RW_state}")
            match self.RW_state:
                case RW_State.Wait:
                    self.StatePub.publish(StateMsg(statemsg = 0))
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
                                self.StatePub.publish(StateMsg(statemsg = 1))
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
                                self.StatePub.publish(StateMsg(statemsg = 2))
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
        self.RWDone = True


    def CachePointAssignment(self):
        while self.CPA.exec == False: 
            match self.CPA_state:
                case CPA_State.DetectCluster:
                    if self.CurrentTypeCA[0] == True:
                        self.CPA.j = self.CurrentTypeCA[1]                
                    if self.VisibleClustersCA[0] == False or self.Pose_O[0] == False:
                        self.CPA_state = CPA_State.DetectCluster
                        self.CPA.exec = True
                    if self.VisibleClustersCA[0] == True and self.Pose_O[0] == True:
                        self.CPA.data = self.VisibleClustersCA[1]
                        self.CPA.counter = 1
                        self.CPA.L.clustersize[0] = 0
                        self.CPA_state = CPA_State.CalculateL
                        #self.CPA.exec = True
                case CPA_State.CalculateL:
                    if self.CPA.counter <= self.CPA.data.m:
                        if (self.CPA.data.clusterdata[self.CPA.counter-1].clustersize[0] > self.CPA.L.clustersize[0]):
                            self.CPA.L = self.CPA.data.clusterdata[self.CPA.counter-1]
                    if self.CPA.counter <= self.CPA.data.m:
                        self.CPA_state = CPA_State.CalculateL
                        self.CPA.counter += 1
                    elif self.CPA.counter > self.CPA.data.m:
                        self.CPA_state = CPA_State.DecideNewCachePoint
                        #self.CPA.exec = True
                case CPA_State.DecideNewCachePoint:
                   
                    if self.CurrentTypeCA[0] == True and (self.CPA.L.clustersize[0] > self.CPA.m[self.CPA.L.clustertype[0]-1].clustersize[0]):
                        self.CPA.j = self.CurrentTypeCA[1]
                        self.CPA_state = CPA_State.UpdateCache
                    elif self.CPA.L.clustersize[0] <= self.CPA.m[self.CPA.L.clustertype[0] -1].clustersize[0]:
                        self.CPA.counter = 0
                        self.CPA_state = CPA_State.UpdateCurrentCache
                    else:
                        self.CPA_state = CPA_State.DetectCluster
                        self.CPA.exec = True
                case CPA_State.UpdateCurrentCache:

                    if (self.CPA.counter <= self.CPA.data.m -1) and (self.CPA.data.clusterdata[self.CPA.counter].clustertype == self.CPA.j):
                        C = self.CPA.data.clusterdata[self.CPA.counter]
                        #translate the cluster to the origin frame
                        # C.centroid[0] = np.cos(self.Pose_O[1][2]) * C.centroid[0] - np.sin(self.Pose_O[1][2]) * C.centroid[1]
                        # C.centroid[1] = np.sin(self.Pose_O[1][2]) * C.centroid[0] + np.cos(self.Pose_O[1][2]) * C.centroid[1]
                        # if self.distance(self.CPA.m[self.CPA.j-1].position, C.position) < self.CPA.threshold: 
                        #     self.CPA.m[self.CPA.j -1].clustersize[0] = max(self.CPA.m[self.CPA.j -1].clustersize[0], C.clustersize[0])
                    if self.CPA.counter <= self.CPA.data.m - 1:
                        self.CPA.counter += 1
                    elif self.CPA.counter > self.CPA.data.m - 1:
                        self.CachePointsCC[0] = True
                        self.CachePointsCC[1] = self.CPA.m
                        self.CachePointsTW[0] = True
                        self.CachePointsTW[1] = self.CPA.m
                        self.CPA_state = CPA_State.DetectCluster
                        self.CPA.exec = True
                case CPA_State.UpdateCache:
                    L = copy.deepcopy(self.CPA.L)
                    self.CPA.m[self.CPA.j -1] = L
                    self.CPA.m[L.clustertype[0] -1].centroid[0] = np.cos(self.Pose_O[1][2]) * L.centroid[0] - np.sin(self.Pose_O[1][2]) * L.centroid[1]
                    self.CPA.m[L.clustertype[0] -1].centroid[1] = np.sin(self.Pose_O[1][2]) * L.centroid[0] + np.cos(self.Pose_O[1][2]) * L.centroid[1]
                        
                    self.CPA.counter = 0
                    self.CPA_state = CPA_State.UpdateCurrentCache

        self.CPADone = True

    def TargetWatch(self):
        while self.TW.exec == False:
            self.get_logger().info(f"state {self.TW_state}")
            match self.TW_state:
                case TW_State.WaitingForTarget:
                    if self.VisibleClustersTW[0] == True:
                        self.TW.data = self.VisibleClustersTW[1]
                    if self.EnableTargetWatch[0] == False:
                        self.TW_state = TW_State.WaitingForTarget
                        self.TW.exec = True

                    elif self.EnableTargetWatch[0] == True and self.CurrentTypeTW[0] == True:
                        self.TW.targetPuckxy = self.EnableTargetWatch[1]
                        self.TW.targetType = self.CurrentTypeTW[1]
                        self.TW.counter = 0
                        self.TW.closestTargetPuck = self.TW.data.puckdata[0]
                        self.TW_state = TW_State.DetectingTarget
                        #self.TW.exec = True
                    elif self.EnableTargetWatch[0] == True and self.VisibleClustersTW[0] == False:
                        self.InvalidTarget_temp = True
                        self.TW_state = TW_State.WaitForDisableTargetWatch
                        self.TW.exec = True
                case TW_State.DetectingTarget | TW_State.CalculateTargetPuck | TW_State.ValidateTargetPuck | TW_State.PublishTargetPuck | TW_State.WaitForDisableTargetWatch:
                    if self.VisibleClustersTW[0] == False:
                        self.TW_state = TW_State.WaitForDisableTargetWatch
                        self.InvalidTarget_temp = True
                        self.TW.exec = True
                    if self.DisableTargetWatch == True:
                        self.TW_state = TW_State.WaitingForTarget
                        self.TW.exec = True
                    match self.TW_state:
                        case TW_State.DetectingTarget:
                            self.TW_state = TW_State.CalculateTargetPuck
                        case TW_State.CalculateTargetPuck:
                            if self.distance(self.TW.data.puckdata[self.TW.counter].position, self.TW.targetPuckxy) <= self.distance(self.TW.closestTargetPuck.position, self.TW.targetPuckxy):
                                self.get_logger().info("here1")
                                self.TW.closestTargetPuck = self.TW.data.puckdata[self.TW.counter]
                            else:
                                self.get_logger().info("did not compute")
                            if self.TW.counter < self.TW.data.n - 1:
                                self.TW.counter += 1
                            elif self.TW.counter == self.TW.data.n - 1 and self.CachePointsTW[0] == True:
                                self.TW.validPuck = True
                                self.TW_state = TW_State.ValidateTargetPuck
                                self.TW.m = self.CachePointsTW[1]
                                self.TW.counter = 0
                            else:
                                self.TW.exec = True
                                
                        case TW_State.ValidateTargetPuck:
                            if self.distance(self.TW.closestTargetPuck.position, self.TW.targetPuckxy) > self.TW.PuckThreshold:
                                self.TW.validPuck = False
                            if self.TW.validPuck == True:
                                self.TW_state = TW_State.PublishTargetPuck
                            elif self.TW.validPuck == False:
                                self.InvalidTarget_temp = True
                                self.get_logger().info("Failed because validPuck is False")
                                #time.sleep(10)
                                self.TW_state = TW_State.WaitForDisableTargetWatch
                        case TW_State.PublishTargetPuck:
                            self.TW.closestTargetPuckPosition = self.TW.closestTargetPuck.position
                            self.TW.counter = 0
                            self.TargetPuck_temp[0] = True
                            self.TargetPuck_temp[1] = self.TW.closestTargetPuckPosition
                            self.TW_state = TW_State.CalculateTargetPuck
                            self.TW.exec = True

                        case TW_State.WaitForDisableTargetWatch:
                            self.InvalidTarget_temp = True
                            self.TW.exec = True
                              
        self.TWDone = True

    def CacheConsS(self):
        while self.CC.exec == False:
            self.get_logger().info(f"state {self.CC_state}")
            match self.CC_state:
                case CC_State.PU_SCAN | CC_State.WaitForCluster |  CC_State.ClusterSeen | CC_State.CalcSmallestVisibleCluster | CC_State.CalculateProb | CC_State.ChooseTargetPosition | CC_State.CalcLeftPuck | CC_State.CalcRightPuck | CC_State.ChooseLeftPuck | CC_State.ChooseRightPuck:
                    if self.CC.done == True:
                       
                        # self.DisableOA_temp = True
                        # self.EnableOA_temp = False
                        self.CC.done = False                 

                        self.CC_state = CC_State.PU_TARGET
                        self.CurrentTypeCA[0] = True
                        self.CurrentTypeCA[1] = self.CC.j
                        self.CurrentTypeTW[0] = True
                        self.CurrentTypeTW[1] = self.CC.j
                        
                        self.EnableClusterWatch_temp = False
                        self.DisableClusterWatch_temp = True
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
                            #self.CC.counter = 0 TODO: check if this is needed
                            self.CC_state = CC_State.WaitForCluster
                        case CC_State.WaitForCluster:
                            if self.VisibleClustersCC[0] == False:
                                self.CC_state = CC_State.WaitForCluster
                                self.CC.exec = True
                            elif self.VisibleClustersCC[0] == True:
                                #self.get_logger().info(f"True2 {self.VisibleClustersCC[1].n}")
                                self.CC.data  = self.VisibleClustersCC[1]
                                self.CC.counter = 0
                                self.CC.SmallestVisibleCluster.clustersize = [1] 
                                #self.DisableClusterWatch_temp = True
                                #self.DisableOA_temp = True
                                self.CCMove_temp[0] = True
                                self.CCMove_temp[1] = (0.0,0.0)
                                self.CC.probDone = False
                                self.CC_state = CC_State.ClusterSeen
                                self.CC.exec = True
                        case CC_State.ClusterSeen | CC_State.CalcSmallestVisibleCluster | CC_State.CalculateProb:
                            #self.get_logger().info("ClusterSeen")
                            # self.get_logger().info(f"randcoef {self.CC.randcoef}")
                            # self.get_logger().info(f"prob {self.CC.prob}")
                            # self.get_logger().info(f"SmallestVisibleCluster {self.CC.SmallestVisibleCluster.clustersize}")
                            if (self.CC.probDone == True) and not (self.CC.randcoef <= self.CC.prob):
                                self.CC_state = CC_State.WaitForCluster
                                self.CC.exec = True
                                # self.CCMove_temp[0] = True TODO: check if this and line below are needed
                                # self.CCMove_temp[1] = (0.0,0.0)
                                #self.CC.j = self.CC.SmallestVisibleCluster.clustertype[0]

                            elif (self.CC.probDone == True) and (self.CC.randcoef <= self.CC.prob): 
                                self.CC_state = CC_State.ChooseTargetPosition
                                self.CC.counter = 0
                                #self.get_logger().info("ChooseTargetPosition")
                            
                            else:
                                match self.CC_state:
                                    case CC_State.ClusterSeen:
                                        self.CC_state = CC_State.CalcSmallestVisibleCluster
                                    case CC_State.CalcSmallestVisibleCluster:
                                        # self.get_logger().info(f"counter{self.CC.counter}")
                                        # self.get_logger().info(f"n{self.CC.data.m}")    
                                        #self.get_logger().info(f"clustersize {self.CC.data.clusterdata[self.CC.counter]}")
                                        if self.CC.data.clusterdata[self.CC.counter].clustersize > self.CC.SmallestVisibleCluster.clustersize:
                                            self.CC.SmallestVisibleCluster = self.CC.data.clusterdata[self.CC.counter]
                                        if self.CC.counter < self.CC.data.m -1:
                                            self.CC.counter += 1
                                        if self.CC.counter == self.CC.data.m - 1:
                                            self.CC_state = CC_State.CalculateProb
                                    case CC_State.CalculateProb:
                                        self.CC.randcoef = randomcoef()
                                        self.CC.prob = (self.CC.k1 / (self.CC.k1 + self.CC.SmallestVisibleCluster.clustersize[0])) * (self.CC.k1 / (self.CC.k1 + self.CC.SmallestVisibleCluster.clustersize[0]))
                                        self.CC.probDone = True
                                        self.get_logger().info(f"prob {self.CC.prob}")
                            
                        case CC_State.ChooseTargetPosition | CC_State.CalcLeftPuck | CC_State.CalcRightPuck | CC_State.ChooseLeftPuck | CC_State.ChooseRightPuck:
                            #self.get_logger().info(f"state {self.CC_state}")
                            match self.CC_state:
                                case CC_State.ChooseTargetPosition:
                                    self.CC_state = CC_State.CalcLeftPuck

                                case CC_State.CalcLeftPuck:
                                    if self.CC.data.puckdata[self.CC.counter].clusterid == self.CC.SmallestVisibleCluster.clusterid and self.CC.data.puckdata[self.CC.counter].positionx < self.CC.leftPuckx:
                                        self.CC.leftPuckx = self.CC.data.puckdata[self.CC.counter].positionx
                                        self.CC.leftPucky = self.CC.data.puckdata[self.CC.counter].positiony
                                        self.CC.leftPuck = self.CC.data.puckdata[self.CC.counter].puckid[0]
                                    if self.CC.counter < self.CC.data.n - 1:
                                        self.CC.counter += 1
                                    if self.CC.counter == self.CC.data.n - 1:
                                        self.CC.counter = 0
                                        self.CC_state = CC_State.CalcRightPuck
                                case CC_State.CalcRightPuck:
                                    if self.CC.data.puckdata[self.CC.counter].clusterid == self.CC.SmallestVisibleCluster.clusterid and self.CC.data.puckdata[self.CC.counter].positionx < self.CC.rightPuckx:
                                        self.CC.rightPuckx = self.CC.data.puckdata[self.CC.counter].positionx
                                        self.CC.rightPucky = self.CC.data.puckdata[self.CC.counter].positiony
                                        self.CC.rightPuck = self.CC.data.puckdata[self.CC.counter].puckid[0]
                                    if self.CC.counter < self.CC.data.n - 1:
                                        self.CC.counter += 1
                                    self.get_logger().info(f"rightPuck {self.CC.rightPuck}")
                                    self.get_logger().info(f"leftPuck {self.CC.leftPuck}")
                                    if self.CC.data.puckdata[self.CC.leftPuck -1].graphorder[0] < self.CC.data.puckdata[self.CC.rightPuck -1].graphorder[0]:
                                        self.CC_state = CC_State.ChooseLeftPuck
                                    elif self.CC.data.puckdata[self.CC.leftPuck -1].graphorder[0] >= self.CC.data.puckdata[self.CC.rightPuck -1].graphorder[0]:
                                        self.CC_state = CC_State.ChooseRightPuck
                                case CC_State.ChooseLeftPuck:
                                        self.CC.targetPosition = [True, (self.CC.leftPuckx, self.CC.leftPucky)]
                                        self.CC.done = True
                                case CC_State.ChooseRightPuck:
                                        self.CC.targetPosition = [True, (self.CC.rightPuckx, self.CC.rightPucky)]
                                        self.CC.done = True
                                        

                case CC_State.PU_TARGET | CC_State.MoveToTarget | CC_State.TurnToTarget | CC_State.LinearMoveToTarget:
                    
                    
                    if self.PuckCarried == True:
                        self.CC.m = self.CachePointsCC[1]  
                        self.CC_state = CC_State.HOMING
                        self.CCMove_temp[0] = True
                        self.CCMove_temp[1] = (0.0,0.0)
                        self.DisableTargetWatch_temp = True
                        self.CC.exec = True
                    match self.CC_state:
                        case CC_State.PU_TARGET:
                            # self.CC.angle = self.angle_between(self.CC.targetPosition[1], self.Pose_O[1])
                            self.CC_state = CC_State.MoveToTarget
                        case CC_State.MoveToTarget | CC_State.TurnToTarget | CC_State.LinearMoveToTarget:
                            match self.CC_state:
                                case CC_State.MoveToTarget:
                                    self.CC_state = CC_State.TurnToTarget
                                case CC_State.TurnToTarget:
                                    self.DisableOA_temp = True
                                    if self.TargetPuck[0] == False:
                                        self.CC.exec = True
                                    if self.CC.angle > 0:
                                        self.CCMove_temp[0] = True
                                        self.CCMove_temp[1] = (0.3*self.CC.av, 0.0)
                                    else:
                                        self.CCMove_temp[0] = True
                                        self.CCMove_temp[1] = (-0.3*self.CC.av, 0.0)
                                    if abs(self.CC.angle - 0.1) > 0.01 > 0.01 and self.Pose_O[0] == True:
                                        target = [0,0]
                                        target[0] = self.TargetPuck[1][0] + (self.Pose_O[1][0])
                                        target[1] = self.TargetPuck[1][1] + (self.Pose_O[1][1])
                                        self.CC.angle = self.calculate_turn_angle(self.Pose_O[1], target)
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
                                    self.CCMove_temp[0] = True
                                    self.CCMove_temp[1] = (0.0, self.CC.lv)
                                    if self.InvalidTarget == True or self.CC.T >= self.CC.timeout:
                                        self.CC_state = CC_State.PU_SCAN
                                        self.DisableTargetWatch_temp = True
                                        self.EnableOA_temp = True
                                        self.CC.exec = True
                                    elif self.InvalidTarget == False and self.CC.T >= self.CC.timeout:
                                        self.CC_state = CC_State.TurnToTarget
                                        target = [0,0]
                                        target[0] = self.TargetPuck[1][0] + (self.Pose_O[1][0])
                                        target[1] = self.TargetPuck[1][1] + (self.Pose_O[1][1])
                                        self.CC.angle = self.calculate_turn_angle(self.Pose_O[1], target)
                                    self.CC.exec = True

                case CC_State.HOMING | CC_State.TurnToHome | CC_State.LinearMoveToHome:
                     #TODO discuss this running in parallel
                    
                    # self.get_logger().info(f"m {self.CC.m}") 
                    # self.get_logger().info(f"j {self.CC.j}") 
                    distance = self.distance(self.CC.m[self.CC.j-1].centroid, (self.Pose_O[1][0], self.Pose_O[1][1]))
                    if distance > 2.0:
                        self.get_logger().info(f"j {self.CC.j}") 
                        self.get_logger().info(f"m {self.CC.m}") 
                    self.get_logger().info(f"Pose_O {self.Pose_O[1]}")
                    self.get_logger().info(f"home distance {self.distance(self.CC.m[self.CC.j-1].centroid, (self.Pose_O[1][0], self.Pose_O[1][1]))}")
                    # if self.PuckCarried == False:
                    #     self.CC_state = CC_State.PU_SCAN
                    #     self.DisableTargetWatch_temp = True
                    #     self.EnableOA_temp = True
                    #     self.CCMove_temp[0] = True
                    #     self.CCMove_temp[1] = (0.0,0.0)
                    #     self.CC.exec = True
                    if self.distance(self.CC.m[self.CC.j-1].centroid, (self.Pose_O[1][0], self.Pose_O[1][1])) < 0.8:
                        self.CC_state = CC_State.DE_PUSH
                        self.CCMove_temp[0] = True
                        self.CCMove_temp[1] = (0.0,0.0)
                        self.CC.T = 0
                        self.CC.exec = True
                    match self.CC_state:
                        case CC_State.HOMING:
                            self.CC_state = CC_State.TurnToHome
                            Pose_O = copy.deepcopy(self.Pose_O)
                            self.CC.angle = self.CC.angle = self.calculate_turn_angle((Pose_O[1][0], Pose_O[1][1], Pose_O[1][2]),(self.CC.m[self.CC.j-1].centroid[0], self.CC.m[self.CC.j-1].centroid[1]))
                        case CC_State.TurnToHome:
                            self.DisableOA_temp = True
                            if self.CC.angle > 0:
                                self.CCMove_temp[0] = True
                                self.CCMove_temp[1] = (0.3*self.CC.av, 0.0)
                            else:
                                self.CCMove_temp[0] = True
                                self.CCMove_temp[1] = (-0.3*self.CC.av, 0.0)
                            if abs(self.CC.angle - 0.1) > 0.01 and self.Pose_O[0]: 
                                Pose_O = copy.deepcopy(self.Pose_O)
                                self.CC.angle = self.calculate_turn_angle((Pose_O[1][0], Pose_O[1][1], Pose_O[1][2]),(self.CC.m[self.CC.j-1].centroid[0], self.CC.m[self.CC.j-1].centroid[1]))
                                self.CC.exec = True
                            elif abs(self.CC.angle - 0.1) <= 0.01 and self.Pose_O[0]: 
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
                                self.CCMove_temp[1] = (0.0, 0.8*self.CC.lv)
                                self.CC.exec = True
                            else:
                                self.CCMove_temp[0] = True
                                self.CCMove_temp[1] = (0.0, 0.0)
                                Pose_O = copy.deepcopy(self.Pose_O)
                                self.CC.angle = self.calculate_turn_angle((Pose_O[1][0], Pose_O[1][1], Pose_O[1][2]),(self.CC.m[self.CC.j-1].centroid[0], self.
                                CC.m[self.CC.j-1].centroid[1]))
                                time.sleep(0.5)
                                self.CC_state = CC_State.TurnToHome
                                self.DisableOA_temp = True
                                self.CC.exec = True
                         

                case CC_State.DE_PUSH:
                    if CC.T > CC.timeout_PUSH:
                        self.CCMove_temp[0] = True
                        self.CCMove_temp[1] = (0.0, 0.2*self.CC.lv)
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
                    if self.PuckCarried == True:
                        msg = Bool()
                        msg.data = True
                        self.Deposit.publish(msg)
                        self.Deposit.publish(msg)
                        self.CC.exec = True
                    elif self.PuckCarried == False and self.CC.T < self.CC.timeout_BACKUP:
                        self.CCMove_temp[0] = True
                        self.CCMove_temp[1] = (0.0, -self.CC.lv)
                        self.CC.exec = True
                    elif self.PuckCarried == False and self.CC.T >= self.CC.timeout_BACKUP:
                        self.CCMove_temp[0] = True
                        self.CCMove_temp[1] = (0.0, 0.0)
                        self.CC.T = 0
                        self.CC_state = CC_State.EXILE
                        self.CC.exec = True
                case CC_State.EXILE | CC_State.EXILE_Turn| CC_State.EXILE_LinearMove:
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
                            
                                

            
        self.CCDone = True

    def MoveManager(self):
        #TODO Need to discuss and fix issue that the MoveManager has to be called only once the other state machines are done. I have put at the end of the callbacks for now as a temporary fix.
        #if self.RWDone == True and self.OADone == True and self.CPADone == True and self.TWDone == True and self.CCDone == True:
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

    def exec(self):
        #update events and clocks for state machines synchronously
        #self.get_logger().info("self.CC.exec" + str(self.CC.exec))
        # self.get_logger().info("self.RW.exec" + str(self.RW.exec))
        # self.get_logger().info("self.MM.exec" + str(self.MM.exec))
        # self.get_logger().info("self.OA.exec" + str(self.OA.exec)) 
        #self.get_logger().info("self.TW.exec" + str(self.TW.exec))
        if self.RW.exec and self.MM.exec and self.OA.exec and self.CC.exec and self.CPA.exec:
            self.get_logger().info("exec")
            #self.get_logger().info(f"state {self.OA_state}")
            #self.get_logger().info(f"state {self.RW_state}")
            #self.get_logger().info(f"state {self.MM_state}")
            #self.get_logger().info(f"state {self.CC_state}")
            #self.get_logger().info(f"state {self.CPA_state}")
            #self.get_logger().info(f"state {self.TW_state}")
            # self.OAMove = self.OAMove_temp
            # self.RWMove = self.RWMove_temp
            # self.CCMove = self.CCMove_temp
            # self.NOAMove = self.NOAMove_temp
            self.MM.T += self.timer_period
            self.OA.T += self.timer_period
            self.RW.T += self.timer_period
            self.CC.T += self.timer_period
            
            self.RW.exec = False
            self.MM.exec = False
            self.OA.exec = False
            self.CC.exec = False
            self.CPA.exec = False
            self.TW.exec = False
            #reset events for next cycle
            self.VisibleClustersCC[0] = False
            self.VisibleClustersCA[0] = False
            self.VisibleClustersTW[0] = False
            self.PuckCarried = False
            self.InvalidTarget = False
            self.EnableClusterWatch = False
            self.DisableClusterWatch = False
            self.EnableTargetWatch[0] = False
            self.DisableTargetWatch = False
            self.EnableOA = True
            self.DisableOA = False
            self.OAMove[0] = False
            self.RWMove[0] = False
            self.CCMove[0] = False
            self.NOAMove[0] = False
            self.TargetPuck[0] = False


            # #updateMoveManagerDones
            # self.OADone = False
            # self.RWDone = False
            # self.CPADone = False
            # self.TWDone = False
            # self.CCDone = False


    

        


               

def main(args=None):
    rclpy.init(args=args)
    node = stateMachine1_real()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()
    