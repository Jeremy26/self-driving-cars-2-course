#!/usr/bin/env python3

"""
Created on 17/11/16
@author: Sam Pfeiffer <sammypfeiffer@gmail.com>
Skeleton of a OpenCV node to subscribe to
either Image or CompressedImage topic and do
some work on the image and publish it
in a Image and CompressedImage topics
(Python has no publisher for doing both
as far as I know).
"""

import rospy
from rostopic import get_topic_type
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np


class OpenCVSkeleton(object):
    def __init__(self):
        # Set CvBridge
        self.bridge = CvBridge()
        # Store last image to process here
        self.last_img = None
        # Also a handy flag if the image is new
        self.is_new_img = False
        # Allow to subscribe to both Image and CompressedImage
        img_in_topicname = rospy.resolve_name('/raspicam_node/image/compressed')
        type_name, img_in_topicname, _ = get_topic_type(img_in_topicname)
        if type_name == 'sensor_msgs/Image':
            self.sub = rospy.Subscriber(img_in_topicname,
                                        Image,
                                        self.img_cb,
                                        queue_size=1)
        elif type_name == 'sensor_msgs/CompressedImage':
            self.sub = rospy.Subscriber(img_in_topicname,
                                        CompressedImage,
                                        self.img_cb,
                                        queue_size=1)
        rospy.loginfo("Subscribing to: " + self.sub.resolved_name +" of type: " + str(self.sub.type))

        img_out_topicname = rospy.resolve_name('/img_out')
        self.pub = rospy.Publisher(img_out_topicname,
                                   Image,
                                   queue_size=1)
        self.pub_compressed = rospy.Publisher(img_out_topicname +
                                              '/compressed',
                                              CompressedImage,
                                              queue_size=1)

    def img_to_cv2(self, image_msg):
        """
        Convert the image message into a cv2 image (numpy.ndarray)
        to be able to do OpenCV operations in it.
        :param Image or CompressedImage image_msg: the message to transform
        """
        rospy.loginfo("image is of type: " + str(type(image_msg)))
        type_as_str = str(type(image_msg))
        if type_as_str.find('sensor_msgs.msg._CompressedImage.CompressedImage') >= 0:
            # Image to numpy array
            np_arr = np.frombuffer(image_msg.data, np.uint8)
            # Decode to cv2 image and store
            return cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        elif type_as_str.find('sensor_msgs.msg._Image.Image') >= 0:
            # Use CvBridge to transform
            try:
                return self.bridge.imgmsg_to_cv2(image_msg,
                                                 image_msg.encoding)  # "bgr8"
            except CvBridgeError as e:
                rospy.logerr("Error when converting image: " + str(e))
                return None
        else:
            rospy.logerr("We don't know how to transform image of type " +
                         str(type(image_msg)) + " to cv2 format.")
            return None

    def img_cb(self, image):
        """
        Callback for the Image or Compressed image subscriber, storing
        this last image and setting a flag that the image is new.
        :param Image or CompressedImage image: the data from the topic
        """
        self.last_img = image
        self.is_new_img = True

    def pub_images(self, cv2_img, image_format="passthrough"):
        """
        Publish onto the Image and CompressedImage topics if there is any
        subscriber.
        :param numpy.ndarray cv2_img: image in cv2 format to publish
        :param str image_format: the image format in which the image should
            be transformed into, list available at:
            http://docs.ros.org/jade/api/sensor_msgs/html/image__encodings_8h_source.html
        """
        if self.pub.get_num_connections() > 0:
            try:
                image_msg = self.bridge.cv2_to_imgmsg(cv2_img, image_format)
                self.pub.publish(image_msg)
            except CvBridgeError as e:
                rospy.logerr("Error on converting image for publishing: " +
                             str(e) + " (Is your image_format correct?)")

        if self.pub_compressed.get_num_connections() > 0:
            msg = CompressedImage()
            msg.header.stamp = rospy.Time.now()
            msg.format = "jpeg"
            msg.data = np.array(cv2.imencode('.jpg', cv2_img)[1]).tostring()
            self.pub_compressed.publish(msg)

    def do_stuff(self):
        """
        Method to do stuff with the last image received.
        First we transform the image message to a cv2 image (numpy.ndarray).
        Then we do OpenCV stuff with it.
        And we publish the new image.
        """
        cv2_img = self.img_to_cv2(self.last_img)
        # Now we can use cv2 functions as the image is <type 'numpy.ndarray'>
        rospy.loginfo("cv2_img: " + str(type(cv2_img)))
        # Your OpenCV stuff
        #
        #

        self.pub_images(cv2_img)
        self.is_new_img = False

    def run(self):
        """
        Method to do stuff at a certain rate.
        """
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.last_img is not None:
                self.do_stuff()
            r.sleep()


if __name__ == '__main__':
    rospy.init_node('camera_test')
    ocvs = OpenCVSkeleton()
    ocvs.run()
    rospy.spin()