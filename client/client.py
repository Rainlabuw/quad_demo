import numpy as np
import rclpy
import threading
import time
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from example_interfaces.msg import String as RosString
from threading import Thread, Event

def threaded(fn):
    def wrapper(*args, **kwargs):
        thread = Thread(target=fn, args=args, kwargs=kwargs)
        thread.start()
        return thread

    return wrapper


class client_main:
    def __init__(self,cf_list):
        self.cf_list = cf_list
        self.cf_mover = {}
        self.state ={}
        self.traj_all = {}
        self.init()
        self.t0 = time.time()
        self.circle()


    def init(self):
        for cf in self.cf_list:
            self.cf_mover[cf] = np.zeros(3)
            self.state[cf] = np.zeros(3)

    @threaded
    def circle(self):
        while True:
            time.sleep(0.01)
            t_elp = time.time() - self.t0
            for cf in self.cf_list:
                self.cf_mover[cf] = np.array([np.cos(t_elp),np.sin(t_elp),1.0])


