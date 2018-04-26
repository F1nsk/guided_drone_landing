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
from pid_controller.pid import PID


def signal_handler(signal, frame):
    print('You pressed Ctrl+C!')
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)

current_pose = SP.PoseStamped()
global new_lower_pos
new_lower_pos = 10000


global pixel_pos
pixel_pos = [0, 0]

UAV_state = mavros_msgs.msg.State()
global last_request
global set_mode


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

def init_drone():
    global last_request
    global set_mode
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

def get_drone_euler():
    quaternion = (
        current_pose.pose.orientation.x,
        current_pose.pose.orientation.y,
        current_pose.pose.orientation.z,
        current_pose.pose.orientation.w)

    euler = euler_from_quaternion(quaternion)
    return euler

def rotate_drone(pid, euler):
    print("Rotate")

    rotation_x = euler[0]
    rotation_y = euler[1]
    rotation_z = euler[2]

    if pixel_pos[0] < 300:
        rotation_z -= pid(pixel_pos[0] - 300)
    else:
        rotation_z -= pid(pixel_pos[0] - 500)

    quaternion = quaternion_from_euler(rotation_x, rotation_y, rotation_z)
    current_pose.pose.orientation.x = quaternion[0]
    current_pose.pose.orientation.y = quaternion[1]
    current_pose.pose.orientation.z = quaternion[2]
    current_pose.pose.orientation.w = quaternion[3]

def left_right_adjust_drone(pid, euler):
    print("Side is close")

    rotation_z = euler[2]
    current_pose.pose.position.x -= pid(pixel_pos[0] - 400) * math.sin(-rotation_z)
    current_pose.pose.position.y -= pid(pixel_pos[0] - 400) * math.cos(-rotation_z)

def up_down_adjust_drone(pid, euler):
    print("Go up or down")

    rotation_z = euler[2]
    current_pose.pose.position.x -= pid(pixel_pos[1] - 400) * math.cos(rotation_z)
    current_pose.pose.position.y -= pid(pixel_pos[1] - 400) * math.sin(rotation_z)

def land_drone(euler):
    global new_lower_pos

    print("Landing")

    rotation_z = euler[2]
    camera_angle = 0.9
    drone_height = current_pose.pose.position.z
    lower_height = 0.5
    ground_dist_to_object = math.tan(camera_angle) * drone_height
    drone_height -= lower_height
    new_ground_dist_to_object = math.tan(camera_angle) * drone_height
    distance_to_move = ground_dist_to_object - new_ground_dist_to_object
    if new_lower_pos == current_pose.pose.position.z:
        current_pose.pose.position.x += distance_to_move * math.cos(rotation_z)
        current_pose.pose.position.y += distance_to_move * math.sin(rotation_z)

    current_pose.pose.position.z -= lower_height
    new_lower_pos = current_pose.pose.position.z


def main():
    global last_request
    global pixel_pos
    global set_mode

    rospy.init_node('pixel_to_pos', anonymous=False)
    rate = rospy.Rate(20)
    mavros.set_namespace('/mavros')

    rospy.Subscriber(mavros.get_topic('state'), mavros_msgs.msg.State, state_callback)
    rospy.Subscriber("pixel_coord", Int16MultiArray, callback_vision, queue_size=1)
    rospy.Subscriber(mavros.get_topic('local_position', 'pose'), mavros.setpoint.PoseStamped, callback_drone,
                     queue_size=1)

    setpoint_local_pub = mavros.setpoint.get_pub_position_local(queue_size=10)

    # /mavros/cmd/arming
    set_arming = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
    # /mavros/set_mode
    set_mode = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)

    while not UAV_state.connected:
        rate.sleep()

    mavros.command.arming(True)

    time.sleep(1)

    set_mode(0, 'OFFBOARD')

    last_request = rospy.Time.now()

    pidHor = PID(0.004, 0.01, 0)
    pidVer = PID(0.006, .0001, 0)

    may_land = False

    while True:
        init_drone()

        euler = get_drone_euler()

        # Marker is too much to the left or right
        if pixel_pos[0] < 300 or pixel_pos[0] > 500:
            rotate_drone(pidHor, euler)

        else:
            # Marker is too left or right
            if pixel_pos[0] < 380 or pixel_pos[0] > 420:
                left_right_adjust_drone(pidVer, euler)

            # Marker is too high or low
            if pixel_pos[1] < 380 or pixel_pos[1] > 420:
                up_down_adjust_drone(pidVer, euler)

        lower_dist = 350
        higher_dist = 450
        if pixel_pos[0] > lower_dist and pixel_pos[0] < higher_dist and pixel_pos[1] > lower_dist and pixel_pos[1] < higher_dist:
            if not may_land:
                land_drone(euler)

                if current_pose.pose.position.z < 0.5:
                    may_land = True

        if may_land:
            print("Going down")
            current_pose.pose.position.z = 0

        setpoint_local_pub.publish(current_pose)
        rate.sleep()

    return 0


if __name__ == '__main__':
    main()
