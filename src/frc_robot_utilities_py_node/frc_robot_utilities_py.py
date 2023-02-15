#!/usr/bin/env python3

import tf2_ros
from tf.transformations import *
import rospy
import nav_msgs.msg
import geometry_msgs.msg
from ck_ros_base_msgs_node.msg import Robot_Status
from ck_ros_msgs_node.msg import HMI_Signals
from frc_robot_utilities_py_node.BufferedROSMsgHandlerPy import BufferedROSMsgHandlerPy
from frc_robot_utilities_py_node.RobotStatusHelperPy import RobotStatusHelperPy, Alliance, RobotMode
from robot_localization.srv import SetPose

from ck_utilities_py_node import geometry, ckmath

hmi_updates = BufferedROSMsgHandlerPy(HMI_Signals)

robot_updates_internal = BufferedROSMsgHandlerPy(Robot_Status)
robot_status = RobotStatusHelperPy(robot_updates_internal)

odom_publisher = rospy.Publisher(name="/ResetHeading", data_class=nav_msgs.msg.Odometry, queue_size=10, tcp_nodelay=True)
set_pose = rospy.ServiceProxy('/set_pose', SetPose)


def register_for_robot_updates():
    global hmi_updates
    global robot_status
    global robot_updates_internal

    hmi_updates.register_for_updates("/HMISignals")
    robot_updates_internal.register_for_updates("/RobotStatus")


def reset_robot_pose_old(x_inches=0, y_inches=0, heading_degrees=0):
    global odom_publisher

    odom = nav_msgs.msg.Odometry()

    odom.header.stamp = rospy.Time.now()
    odom.header.frame_id = 'odom'
    odom.child_frame_id = 'base_link'

    # odom.pose.pose = geometry.Pose().to_msg()
    position = geometry.Translation()
    position.x = ckmath.inches_to_meters(x_inches)
    position.y = ckmath.inches_to_meters(y_inches)
    odom.pose.pose.position = position.to_msg()

    orientation = geometry.Rotation()
    orientation.yaw = math.radians(heading_degrees)
    odom.pose.pose.orientation = orientation.to_msg_quat()

    odom.twist.twist = geometry.Twist().to_msg()

    pose_covariance = geometry.Covariance()
    # pose_covariance.x_var = 0.001
    # pose_covariance.y_var = 0.001
    # pose_covariance.z_var = 0.001
    # pose_covariance.yaw_var = 0.001
    # pose_covariance.pitch_var = 0.001
    # pose_covariance.roll_var = 0.001

    odom.pose.covariance = pose_covariance.to_msg()
    odom.twist.covariance = geometry.Covariance().to_msg()

    odom_publisher.publish(odom)

def reset_robot_pose(x_inches=0, y_inches=0, heading_degrees=0):
    global set_pose

    odom = geometry_msgs.msg.PoseWithCovarianceStamped()

    odom.header.stamp = rospy.Time.now()
    odom.header.frame_id = 'odom'
    
    pose = geometry.Pose()
    pose.position.x = ckmath.inches_to_meters(x_inches)
    pose.position.y = ckmath.inches_to_meters(y_inches)
    pose.orientation.yaw = math.radians(heading_degrees)

    odom.pose.pose = pose.to_msg()

    pose_covariance = geometry.Covariance()
    pose_covariance.x_var = 0.01
    pose_covariance.y_var = 0.01
    pose_covariance.yaw_var = math.radians(1)

    odom.pose.covariance = pose_covariance.to_msg()

    set_pose(odom)
