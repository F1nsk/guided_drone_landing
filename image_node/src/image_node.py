#!/usr/bin/env python
# import ROS libraries
import rospy
import sys
import signal
from geometry_msgs.msg import Vector3
import math
#ROS messaging libraries
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy
import argparse
from std_msgs.msg import String
from std_msgs.msg import Int16MultiArray



def main():
    rospy.init_node('image_seg_node', anonymous=False)
    #rate = rospy.Rate(20)       # 20 packages pr. sec
    img_sub = rospy.Subscriber("iris/camera/image_raw",Image,callback,queue_size=1)



def pub(data):
    img_pub = rospy.Publisher("pixel_coord", Int16MultiArray,queue_size=1)
    img_pub.publish(data)

def callback(img):
    #rospy.loginfo(rospy.get_caller_id())
    bridge = CvBridge()
    bgr_img = bridge.imgmsg_to_cv2(img, "bgr8")
    upper =  numpy.array([0,0,255])
    lower = numpy.array([0,0,0])
    binImg = cv2.inRange(bgr_img,lower,upper)
    #cv2.imwrite("test.png",binImg)
    #cv2.imshow("binary img",binImg)
    #cv2.waitKey(2)

    contours = 0
    # Finding contours
    contours = cv2.findContours(binImg,mode=cv2.RETR_LIST,method=cv2.CHAIN_APPROX_NONE)

    t=cv2.moments(binImg)
    #rospy.loginfo(t)
    cX = -1
    cY = -1
    if t["m00"]!= 0:
        # compute center of mass
        cX = int(t["m10"] / t["m00"])
        cY = int(t["m01"] / t["m00"])



    # draw the center of the shape on the image
    cv2.circle(bgr_img, (cX, cY), 7, (255, 255, 255), -1)

    array = [cX,cY]
    my_array_for_publishing = Int16MultiArray(data=array)
    pub(my_array_for_publishing)
    rospy.loginfo("msg sendt")

    #cv2.imshow("center of mass",bgr_img)
    #cv2.waitKey(2)

if __name__ == '__main__':
    main()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("shutting down")