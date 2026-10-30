

# Import the necessary libraries
import os
import sys
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
import rclpy  # Python library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
import time
import numpy as np
import copy
from interfaces.msg import PoseO, PuckCarried, VisibleClustersCC, PuckData, ClusterData, Pick, Place
import RPi.GPIO as GPIO

class PickPlace(Node):
    """ 
    Create an ImageSubscriber class, which is a subclass of the Node class.
    """

    def __init__(self):
        """
        Class constructor to set up the node
        """
        # Initiate the Node class's constructor and give it a name
        super().__init__('PickPlace')

        

        #Puck Carried publisher
        self.PuckCarried_pub = self.create_publisher(PuckCarried, 'PuckCarried', 10)

        #Subscriber
        self.Pick_sub = self.create_subscription(Pick, 'Pick', self.Pick_callback,1)
        self.Place_sub = self.create_subscription(Place, 'Place', self.Place_callback,1)


    def Place_callback(self, msg):
        print(msg)
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(11, GPIO.OUT)
        servo = GPIO.PWM(11, 50)
        servo.start(0)
        time.sleep(1)
        servo.ChangeDutyCycle(3.7)
        time.sleep(1)
        time.sleep(1)
        servo.stop()
        GPIO.cleanup()
        pub_msg = PuckCarried()
        pub_msg.puckcarried = True
        self.PuckCarried_pub.publish(pub_msg)
        print("placed")

    def Pick_callback(self, msg):
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(11, GPIO.OUT)
        servo = GPIO.PWM(11, 50)
        servo.start(0)
        time.sleep(1)
        servo.ChangeDutyCycle(7.0)
        time.sleep(1)
        servo.stop()
        GPIO.cleanup()
        pub_msg = PuckCarried()
        pub_msg.puckcarried = False
        self.PuckCarried_pub.publish(pub_msg)
        print("picked")

def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    pickplace = PickPlace()

    # Spin the node so the callback function is called.

    print("spin PickPlace")
    rclpy.spin(pickplace)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pickplace.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()


if __name__ == '__main__':
    main()
