#!/usr/bin/env python3

import tf2_ros
import rospy
from rio_control_node.msg import Motor_Status, Robot_Status
from hmi_agent_node.msg import HMI_Signals
from BufferedROSMsgHandlerPy import BufferedROSMsgHandlerPy
from RobotStatusHelperPy import RobotStatusHelperPy

hmi_updates = BufferedROSMsgHandlerPy(HMI_Signals)

robot_updates_internal = BufferedROSMsgHandlerPy(Robot_Status)
robot_status = RobotStatusHelperPy(robot_updates_internal)

motor_updates_internal = BufferedROSMsgHandlerPy(Motor_Status)
# MotorStatusHelper motor_updates(motor_updates_internal);

def register_for_robot_updates(node_handle):
    hmi_updates.register_for_updates("/HMISignals")
    robot_updates_internal.register_for_updates("/RobotStatus")
    motor_updates_internal.register_for_updates("/MotorStatus")