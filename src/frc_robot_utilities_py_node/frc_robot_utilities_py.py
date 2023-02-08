#!/usr/bin/env python3

import tf2_ros
from tf.transformations import *
import rospy
from nav_msgs.msg import Odometry
from ck_ros_base_msgs_node.msg import Robot_Status
from ck_ros_msgs_node.msg import HMI_Signals
from frc_robot_utilities_py_node.BufferedROSMsgHandlerPy import BufferedROSMsgHandlerPy
from frc_robot_utilities_py_node.RobotStatusHelperPy import RobotStatusHelperPy, Alliance, RobotMode

from ck_utilities_py_node.geometry import *
from ck_utilities_py_node.ckmath import *

hmi_updates = BufferedROSMsgHandlerPy(HMI_Signals)

robot_updates_internal = BufferedROSMsgHandlerPy(Robot_Status)
robot_status = RobotStatusHelperPy(robot_updates_internal)

odometry_publisher = rospy.Publisher(name="/ResetHeading", data_class=Odometry, queue_size=10, tcp_nodelay=True)


def register_for_robot_updates():
    global hmi_updates
    global robot_status
    global robot_updates_internal

    hmi_updates.register_for_updates("/HMISignals")
    robot_updates_internal.register_for_updates("/RobotStatus")


def reset_robot_pose(x_inches=0, y_inches=0, heading_degrees=0):
    global odometry_publisher

    odom = Odometry()

    odom.header.stamp = rospy.Time.now()
    odom.header.frame_id = 'odom'
    odom.child_frame_id = 'base_link'

    # position = Translation()
    # position.x = x
    # position.y = y
    # position.z = 0

    # orientation = Rotation()
    # orientation.roll = 0
    # orientation.pitch = 0
    # orientation.yaw = math.radians(heading)

    # linear = Translation()
    # angular = Rotation()
    
    # odom.pose.pose.position = position.to_msg()
    # odom.pose.pose.orientation = orientation.to_msg()
    # odom.twist.twist.linear = linear.to_msg()
    # odom.twist.twist.angular = angular.to_msg()

    odom.pose.pose.position.x = inches_to_meters(x_inches)
    odom.pose.pose.position.y = inches_to_meters(y_inches)
    odom.pose.pose.position.z = 0
    
    quat = quaternion_from_euler(0, 0, math.radians(heading_degrees))
    odom.pose.pose.orientation.x = quat[0]
    odom.pose.pose.orientation.y = quat[1]
    odom.pose.pose.orientation.z = quat[2]
    odom.pose.pose.orientation.w = quat[3]

    odom.twist.twist.linear.x = 0
    odom.twist.twist.linear.y = 0
    odom.twist.twist.linear.z = 0
    odom.twist.twist.angular.x = 0
    odom.twist.twist.angular.y = 0
    odom.twist.twist.angular.z = 0

    odom.pose.covariance = [
        0.001,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.001,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.001,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.001,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.001,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.00001,
    ]

    odom.twist.covariance =[
        0.001,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.001,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.001,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.001,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.001,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.001,
    ]

    odometry_publisher.publish(odom)
