#!/usr/bin/env python

import rospy
import sys
import signal
from markerlocator.MarkerTracker import MarkerTracker
#ROS messaging libraries
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
from std_msgs.msg import Float32MultiArray


def main():
    rospy.init_node('MarkerLocator', anonymous=False)
    img_sub = rospy.Subscriber("iris/camera/image_raw", Image, callback, queue_size=1)

def pub(data):
    img_pub = rospy.Publisher("marker_pos", Float32MultiArray,queue_size=1)
    img_pub.publish(data)

def callback(data):
    # Marker
    order = 6
    kernelsize = 3
    scale = 0.5

    bridge = CvBridge()
    img = bridge.imgmsg_to_cv2(data,"bgr8")
    grey = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

    ml = MarkerTracker(order,kernelsize,scale)
    pos = ml.locate_marker(grey)
    #[x ,y, ori, q, ori]  = pos
    x = pos.x
    y = pos.y
    Q =pos.quality
    theta = pos.theta

    #rospy.loginfo("Anders er en mindre idiot end foer..")
    #rospy.loginfo(x)
    array = [x,y,theta,Q]
    my_array_for_publishing = Float32MultiArray(data=array)
    pub(my_array_for_publishing)
    rospy.loginfo("msg sendt")


if __name__ == '__main__':
    main()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("shutting down")
