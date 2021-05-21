#!/usr/bin/env python3
from PCA9685 import PCA9685
from Motor import *
from Ultrasonic import *
import rospy
from sensor_msgs.msg import Image
from time import sleep
import cv2
import numpy as np
import os

class Controller(object):
    def __init__(self):
        self.setup_ros()
        self.loop()

    def setup_ros(self):
        """
        Build the node, the subscriber, and the publisher
        """
        rospy.init_node('controller')
        #rospy.Subscriber("/image_raw", Image, self.image_callback, queue_size =1)
        #self.pub = rospy.Publisher('/planner', Image, queue_size=1)

    def image_callback(self,msg):
        """
        Build the image callback
        """
        self.image = msg

    def loop(self):
        """
        Define the code that runs.
        Publishes another topic
        """
        rate = rospy.Rate(15)

        while not rospy.is_shutdown():
            motor = Motor()
            ultrasonic = Ultrasonic()
            print(ultrasonic.get_distance())
            if ultrasonic.get_distance()>20:
                motor.move(1000,1000,1000,1000)
            else:
                motor.move(0,0,0,0)
            rate.sleep()
  
if __name__ == '__main__':
    try:
        Controller()
    except KeyboardInterrupt:
        motor = Motor()
        motor.move(0,0,0,0)
        motor.destroy()