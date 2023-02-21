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
from robot_localization.srv import SetPose, SetPoseRequest

from ck_utilities_py_node.geometry import *
from ck_utilities_py_node.ckmath import *

hmi_updates = BufferedROSMsgHandlerPy(HMI_Signals)

robot_updates_internal = BufferedROSMsgHandlerPy(Robot_Status)
robot_status = RobotStatusHelperPy(robot_updates_internal)

set_pose = rospy.ServiceProxy('/set_pose', SetPose)


def register_for_robot_updates():
    global hmi_updates
    global robot_status
    global robot_updates_internal

    hmi_updates.register_for_updates("/HMISignals")
    robot_updates_internal.register_for_updates("/RobotStatus")

def reset_robot_pose(alliance : Alliance, x_inches=0, y_inches=0, heading_degrees=0):
    global set_pose

    odom = geometry_msgs.msg.PoseWithCovarianceStamped()

    odom.header.stamp = rospy.Time.now()
    odom.header.frame_id = 'odom'

    pose = Pose()
    pose.position.x = inches_to_meters(x_inches)
    pose.position.y = inches_to_meters(y_inches)
    pose.orientation.yaw = math.radians(heading_degrees)

    if alliance == Alliance.BLUE:
        pose.orientation.yaw += math.pi

    odom.pose.pose = pose.to_msg()

    pose_covariance = Covariance()
    pose_covariance.x_var = 0.01
    pose_covariance.y_var = 0.01
    pose_covariance.yaw_var = math.radians(1)

    odom.pose.covariance = pose_covariance.to_msg()

    set_pose_msg = SetPoseRequest()
    set_pose_msg.pose = odom

    set_pose(set_pose_msg)
