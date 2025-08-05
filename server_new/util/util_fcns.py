import time
import numpy as np
from threading import Thread, Event
import threading
from scipy.spatial.transform import Rotation as rot
import global_constants as const
def threaded(fn):
    def wrapper(*args, **kwargs):
        thread = Thread(target=fn, args=args, kwargs=kwargs)
        thread.start()
        return thread

    return wrapper


def box_constratins(mover_i):
    if mover_i[0] >= const.x_max:
        mover_i[0] = const.x_max
    elif mover_i[0] <= const.x_min:
        mover_i[0] = const.x_min
    if mover_i[1] >= const.y_max:
        mover_i[1] = const.y_max
    elif mover_i[1] <= const.y_min:
        mover_i[1] = const.y_min
    if mover_i[2] >= const.z_max:
        mover_i[2] = const.z_max
    elif mover_i[2] <= const.z_min:
        mover_i[2] = const.z_min
    return mover_i


def q2e(q: np.ndarray):
    r = rot.from_quat(q)
    e = r.as_euler("xyz", degrees=True)
    return e

def e2q(e: np.ndarray):
    r = rot.from_euler("xyz", e, degrees=True)
    q = r.as_quat()
    return q

