import rospy
from threading import Lock

class BufferedROSMsgHandlerPy:
    def __init__(self, data_class):
        self.__update_occurred = False
        self.__mutex = Lock()
        self.__msg_update_tmp = None
        self.__msg_buf = None
        self.__data_class = data_class

    def __update_func(self, msg):
        self.__mutex.acquire()
        try:
            self.__msg_update_tmp = msg
            self.__update_occurred = True
        finally:
            self.__mutex.release()

    
    def register_for_updates(self, topic_name):
        rospy.Subscriber(name=topic_name, data_class=self.__data_class, callback=self.__update_func, queue_size=1, tcp_nodelay=True)
    
    def get(self):
        if(self.__update_occurred):
            self.__mutex.acquire()
            try:
                self.__msg_buf = self.__msg_update_tmp
                self.__update_occurred = False
            finally:
                self.__mutex.release()
        return self.__msg_buf
    
    def has_updated(self)->bool:
        return self.__update_occurred
    
