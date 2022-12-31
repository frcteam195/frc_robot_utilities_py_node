import rospy
from threading import Lock
from typing import TypeVar, Generic
from frc_robot_utilities_py_node.BufferedROSMsgHandlerPy import BufferedROSMsgHandlerPy

ControlClass = TypeVar('ControlClass')
StatusClass = TypeVar('StatusClass')

class SubsystemController(Generic[ControlClass, StatusClass]):

    def __init__(self, control_topic : str, status_topic : str):
        self.__publisher = rospy.Publisher(name=control_topic, data_class=ControlClass, queue_size=50, tcp_nodelay=True)
        self.__status_handler = BufferedROSMsgHandlerPy(StatusClass)
        self.__status_handler.register_for_updates(status_topic)

    def get(self) -> StatusClass:
        return self.__status_handler.get()

    def publish(self, msg : ControlClass):
        if isinstance(msg, ControlClass):
            self.__publisher.publish(msg)