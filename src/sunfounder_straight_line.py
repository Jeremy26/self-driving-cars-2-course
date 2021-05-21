#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from picar import front_wheels, back_wheels
from picar.SunFounder_PCA9685 import Servo
import picar
from time import sleep
import cv2
import numpy as np
import os

class Planner(object):
    def __init__(self):
        self.setup_ros()
        self.loop()

    def setup_ros(self):
        """
        Build the node, the subscriber, and the publisher
        """
        rospy.init_node('planner')
        #rospy.Subscriber("/image_raw", Image, self.image_callback, queue_size =1)
        #self.pub = rospy.Publisher('/planner', Image, queue_size=1)

    def image_callback(self,msg):
        """
        Build the image callback
        """
        self.image = msg

    def destroy(self):
        self.bw.stop()
        self.fw.stop()

    def loop(self):
        """
        Define the code that runs.
        Publishes another topic
        """
        rate = rospy.Rate(15)

        while not rospy.is_shutdown():
            #self.pub.publish(self.image)
            picar.setup()
            self.bw = back_wheels.Back_Wheels()
            self.fw = front_wheels.Front_Wheels()
            #pan_servo = Servo.Servo(1)
            #tilt_servo = Servo.Servo(2)
            picar.setup()
            self.fw.offset = 0
            #pan_servo.offset = 10
            #tilt_servo.offset = 0

            self.fw.turn(30)
            #pan_servo.write(90)
            #tilt_servo.write(90)
            self.bw.speed = 20
            #self.fw.speed = 20
            self.bw.forward()
            rate.sleep()
  
if __name__ == '__main__':
    try:
        Planner()
    except KeyboardInterrupt:
        self.destroy()