import numpy as np
from .util import global_constants as const



def compute_velocity(error_history,name):
    kp = const.P_kp
    kd = const.P_kd
    v_max = const.v_max
    v_des = kp * error_history[name][-1, 0:3] + kd * (
            error_history[name][-1, 0:3] - error_history[name][-2, 0:3]) / 0.02
    for i in range(len(v_des)):
        if v_des[i] >= v_max:
            v_des[i] = v_max
        elif v_des[i] <= -v_max:
            v_des[i] = -v_max
    return v_des