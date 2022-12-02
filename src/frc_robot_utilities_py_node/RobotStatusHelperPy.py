from enum import Enum
from threading import Lock
from BufferedROSMsgHandlerPy import BufferedROSMsgHandlerPy

class Alliance(Enum):
    RED = 0
    BLUE = 1
    UNKNOWN = 2

class RobotMode(Enum):
    DISABLED = 0
    TELEOP = 1
    AUTONOMOUS = 2
    TEST = 3

class RobotStatusHelperPy:
    def __init__(self, buffered_msg_obj : BufferedROSMsgHandlerPy):
        self.__bufmsgobj = buffered_msg_obj
        self.__robot_state : RobotMode = RobotMode.DISABLED
        self.__alliance : Alliance = Alliance.UNKNOWN
        self.__match_time = 0
        self.__game_data = ""
        self.__selected_auto = 0
        self.__mutex = Lock()

    def __update(self):
        if(self.__bufmsgobj.has_updated()):
            self.__mutex.acquire()
            try:
                r_stat = self.__bufmsgobj.get()
                self.__robot_state = RobotMode(r_stat.robot_state)
                self.__alliance = Alliance(r_stat.alliance)
                self.__match_time = r_stat.match_time
                self.__game_data = r_stat.game_data
                self.__selected_auto = r_stat.selected_auto
            finally:
                self.__mutex.release()

    def get_mode(self)->RobotMode:
        self.__update()
        return self.__robot_state

    def get_alliance(self)->Alliance:
        self.__update()
        return self.__alliance

    def get_match_time(self):
        self.__update()
        return self.__match_time

    def get_game_data(self):
        self.__update()
        return self.__game_data

    def get_selected_auto(self):
        self.__update()
        return self.__selected_auto