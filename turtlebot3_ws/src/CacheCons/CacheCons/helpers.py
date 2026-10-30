"""Shared constructors and random helpers of the CacheCons state machines."""
from geometry_msgs.msg import Twist
from interfaces.msg import ClusterData
import numpy as np


def randomcoef():
    return np.random.uniform(0, 5)


def random_sign():
    return np.random.choice([-1, 1])


def randomnat():
    return np.random.randint(0, 5)


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
