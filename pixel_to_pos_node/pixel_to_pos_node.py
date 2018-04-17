#!/usr/bin/env python

# import ROS libraries
import rospy
import mavros
from mavros.utils import *
from mavros import setpoint as SP
import mavros.setpoint
import mavros.command
import mavros_msgs.msg
import mavros_msgs.srv
import sys
import signal
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Int16MultiArray
from geometry_msgs.msg import Vector3
import math
import time

def signal_handler(signal, frame):
    print('You pressed Ctrl+C!')
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)

current_pose = SP.PoseStamped()

global pixel_pos
pixel_pos = [0, 0]

UAV_state = mavros_msgs.msg.State()


def state_callback(topic):
    UAV_state.armed = topic.armed
    UAV_state.connected = topic.connected
    UAV_state.mode = topic.mode
    UAV_state.guided = topic.guided


def callback_vision(topic):
    global pixel_pos
    pixel_pos = topic.data




def callback_drone(topic):
    current_pose.pose = topic.pose


def main():
    rospy.init_node('pixel_to_pos', anonymous=False)
    rate = rospy.Rate(20)
    mavros.set_namespace('/mavros')

    rospy.Subscriber(mavros.get_topic('state'), mavros_msgs.msg.State, state_callback)
    rospy.Subscriber("pixel_coord", Int16MultiArray, callback_vision, queue_size=1)
    rospy.Subscriber(mavros.get_topic('local_position', 'pose'), mavros.setpoint.PoseStamped, callback_drone, queue_size=1)

    setpoint_local_pub = mavros.setpoint.get_pub_position_local(queue_size=10)

    # /mavros/cmd/arming
    set_arming = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
    # /mavros/set_mode
    set_mode = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)

    global pixel_pos

    while not UAV_state.connected:
        rate.sleep()

    mavros.command.arming(True)

    time.sleep(1)

    set_mode(0, 'OFFBOARD')

    last_request = rospy.Time.now()

    while True:

        if (UAV_state.mode != "OFFBOARD" and
                (rospy.Time.now() - last_request > rospy.Duration(5.0))):
            set_mode(0, 'OFFBOARD')
            print("enabling offboard mode")
            last_request = rospy.Time.now()
        else:
            if (not UAV_state.armed and
                    (rospy.Time.now() - last_request > rospy.Duration(5.0))):
                if (mavros.command.arming(True)):
                    print("Vehicle armed")
                last_request = rospy.Time.now()

        quaternion = (
            current_pose.pose.orientation.x,
            current_pose.pose.orientation.y,
            current_pose.pose.orientation.z,
            current_pose.pose.orientation.w)

        euler = euler_from_quaternion(quaternion)
        rotation_x = euler[0]
        rotation_y = euler[1]
        rotation_z = euler[2]

        rospy.loginfo("x: %f" % rotation_x)
        rospy.loginfo("y: %f" % rotation_y)
        rospy.loginfo("z: %f" % rotation_z)

        # Marker is too much to the left or right
        if pixel_pos[0] < 390:
            rotation_z = rotation_z + 0.1
            quaternion = quaternion_from_euler(rotation_x, rotation_y, rotation_z)
            current_pose.pose.orientation.x = quaternion[0]
            current_pose.pose.orientation.y = quaternion[1]
            current_pose.pose.orientation.z = quaternion[2]
            current_pose.pose.orientation.w = quaternion[3]
        elif pixel_pos[0] > 410:
            rotation_z = rotation_z - 0.1
            quaternion = quaternion_from_euler(rotation_x, rotation_y, rotation_z)
            current_pose.pose.orientation.x = quaternion[0]
            current_pose.pose.orientation.y = quaternion[1]
            current_pose.pose.orientation.z = quaternion[2]
            current_pose.pose.orientation.w = quaternion[3]

        # Marker is too high or low
        if pixel_pos[1] < 390:
            current_pose.pose.position.x += 0.5 * math.cos(rotation_z)
            current_pose.pose.position.y += 0.5 * math.sin(rotation_z)

        elif pixel_pos[1] > 410:
            current_pose.pose.position.x -= 0.5 * math.cos(rotation_z)
            current_pose.pose.position.y -= 0.5 * math.sin(rotation_z)
            
        else:
            quaternion = quaternion_from_euler(0, 0, rotation_z)
            current_pose.pose.orientation.x = quaternion[0]
            current_pose.pose.orientation.y = quaternion[1]
            current_pose.pose.orientation.z = quaternion[2]
            current_pose.pose.orientation.w = quaternion[3]

        current_pose.pose.position.z = 5
        setpoint_local_pub.publish(current_pose)
        rate.sleep()

    return 0


if __name__ == '__main__':
    main()
