#!/usr/bin/env python3

import tf2_ros
import rospy
from rio_control_node.msg import Motor_Status, Robot_Status
from hmi_agent_node.msg import HMI_Signals
from frc_robot_utilities_py_node.BufferedROSMsgHandlerPy import BufferedROSMsgHandlerPy
from frc_robot_utilities_py_node.RobotStatusHelperPy import RobotStatusHelperPy

hmi_updates = BufferedROSMsgHandlerPy(HMI_Signals)

robot_updates_internal = BufferedROSMsgHandlerPy(Robot_Status)
robot_status = RobotStatusHelperPy(robot_updates_internal)

motor_updates_internal = BufferedROSMsgHandlerPy(Motor_Status)
# MotorStatusHelper motor_updates(motor_updates_internal);

def register_for_robot_updates():
    global hmi_updates
    global robot_status
    global robot_updates_internal
    global motor_updates_internal
    
    hmi_updates.register_for_updates("/HMISignals")
    robot_updates_internal.register_for_updates("/RobotStatus")
    motor_updates_internal.register_for_updates("/MotorStatus")