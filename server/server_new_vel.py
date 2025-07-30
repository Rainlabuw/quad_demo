import time
import numpy
import threading
import cflib.crtp as crtp
import motioncapture
import numpy as np
from scipy.spatial.transform import Rotation as rot
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from threading import Thread, Event
from numpy import linalg as LA
from scipy.spatial.transform import Rotation as rot


def threaded(fn):
    def wrapper(*args, **kwargs):
        thread = Thread(target=fn, args=args, kwargs=kwargs)
        thread.start()
        return thread

    return wrapper


## global variables
n = 8  ## system states
m = 3 + 4  ## input position and quaternion (0,0,0,1)
data_length = 20


class TrackingObject:
    uri = "UNKNOWN"
    object_name = "UNKNOWN"

    def __init__(self, uri, object_name):
        self.uri = uri
        self.object_name = object_name


class CrazyflieServer:

    def __init__(self, crazyflies=[], vicon_ip="", kill_sleep=20):
        self.duration = 1200.0
        self.save_flag = True
        self.print_t = 50.0
        self.lock = threading.Lock()
        self.is_open = True
        self.Control_Flag = False
        self.log_data = "entity,time,x,y,z,p,r,t,w\n"
        self.curr_data = {}
        self.data_history = {}
        self.error_history = {}
        self.threads = []
        self.crazyflies = crazyflies
        ## create name list
        self.cf_list = []
        for cf in self.crazyflies:
            self.cf_list.append(cf.object_name)
        self.mover = {}
        self.mover_history = {}
        self.hover = {}
        self.hover_flag = {}
        ## initialize the data
        self.initialization()
        self.record_count = 0.0
        self.record_count_ekf = 0.0
        self.time_list_vicon = np.zeros(10)
        self.time_list_ekf = np.zeros(10)
        self.time_list_control = np.zeros(10)
        print("1) Connecting to Crazyflies_Justin_vel")
        crtp.init_drivers()
        log_config = LogConfig(name="pos_est", period_in_ms=10)
        # log_config.add_variable( 'kalman.stateX', 'float' )
        # log_config.add_variable( 'kalman.stateY', 'float' )
        # log_config.add_variable( 'kalman.stateZ', 'float' )
        log_config.add_variable('stateEstimate.x', 'float')
        log_config.add_variable('stateEstimate.y', 'float')
        log_config.add_variable('stateEstimate.z', 'float')

        log_config2 = LogConfig(name="att_est", period_in_ms=10)
        log_config2.add_variable('stateEstimate.qx', 'float')
        log_config2.add_variable('stateEstimate.qy', 'float')
        log_config2.add_variable('stateEstimate.qz', 'float')
        log_config2.add_variable('stateEstimate.qw', 'float')

        log_config3 = LogConfig(name="euler_est", period_in_ms=10)
        log_config3.add_variable('stateEstimate.vx', 'float')
        log_config3.add_variable('stateEstimate.vy', 'float')
        log_config3.add_variable('stateEstimate.vz', 'float')

        for cf in self.crazyflies:
            print(f"Connecting crazyflie {cf.object_name} to radio: {cf.uri}")
            cf.cf = Crazyflie()
            cf.cflog_pos = SyncLogger(cf.cf, log_config)
            cf.cflog_att = SyncLogger(cf.cf, log_config2)
            cf.cflog_vel = SyncLogger(cf.cf, log_config3)
            cf.cf.open_link(cf.uri)
        time.sleep(1.0)
        print("2) Initialzing to Telemetry System")
        print(f"Connecting to Vicon with IP: {vicon_ip}")
        self.vicon_ip = vicon_ip
        self.t0 = time.time()
        self.vicon()
        # self.spawn_thread( self.vicon )
        time.sleep(1.0)
        print("3) Initializing Crazyflies Kalman Estimator")
        kvar_cfg = LogConfig(name="var", period_in_ms=500)
        kvar_cfg.add_variable('kalman.varPX', 'float')
        kvar_cfg.add_variable('kalman.varPY', 'float')
        kvar_cfg.add_variable('kalman.varPZ', 'float')

        for cf in self.crazyflies:
            cf.cf.param.set_value('stabilizer.estimator', '2')
            cf.cf.param.set_value('locSrv.extQuatStdDev', 0.06)
            cf.cf.param.set_value('kalman.resetEstimation', '1')
            ## control limits
            # cf.cf.param.set_value('posCtlPid.rLimit',10.0)
            # cf.cf.param.set_value('posCtlPid.pLimit',10.0)
            # cf.cf.param.set_value('posCtlPid.zVelMax',0.3)
            cf.kvar_log = SyncLogger(cf.cf, kvar_cfg)
            cf.var_history = [[1000] * 10, [1000] * 10, [1000] * 10]
            cf.kvar_log.connect()
        print("4) Awaiting Kalman Estimator")
        var_active = True
        iteration = 0
        while var_active:
            var_active = False
            for cf in self.crazyflies:
                for log_entry in cf.kvar_log:
                    data = log_entry[1]
                    cf.var_history[0][iteration] = data['kalman.varPX'];
                    cf.var_history[1][iteration] = data['kalman.varPY'];
                    cf.var_history[2][iteration] = data['kalman.varPZ'];
                    threshold = 0.01
                    threshold = 1
                    print(f"Kalman variance: {numpy.amax(cf.var_history) - numpy.amin(cf.var_history)}")
                    if (numpy.amax(cf.var_history) - numpy.amin(cf.var_history)) > threshold:
                        var_active = True
                    break
            iteration = (iteration + 1) % 10
        time.sleep(1.0)
        print("5) Arming Drones")
        for cf in self.crazyflies:
            cf.cf.param.set_value('kalman.resetEstimation', '0')
            cf.kvar_log.disconnect()
        time.sleep(1.0)
        for cf in self.crazyflies:
            cf.cflog_pos.connect()
            cf.cflog_att.connect()
            cf.cflog_vel.connect()
            cf.cf.platform.send_arming_request(True)
            # cf.com = MotionCommander( cf.cf, 0.5 )
        # self.spawn_thread( self.logging )
        self.logging()
        time.sleep(1.0)
        self.kill_sleep = kill_sleep
        self.spawn_thread(self.killswitch)
        print("6) All ready!")
        time.sleep(3.0)
        self.start_time = time.time()
        self.ros_bridge_execute(self.Control_Flag)

    def spawn_thread(self, func):
        thread = threading.Thread(target=func)
        thread.start()
        self.threads.append(thread);

    def initialization(self):
        for cf in self.cf_list:
            self.mover[cf] = np.zeros(m)
            self.mover[cf][-1] = 1.0  ## scaler quaternion
            self.mover_history[cf] = np.zeros((data_length, m))
            self.hover[cf] = np.zeros(3)
            self.hover_flag[cf] = False
            self.data_history[cf] = np.zeros((data_length, 13))
            self.error_history[cf] = np.zeros((data_length, 13))

    # @threaded
    def vel_controller(self, cf):
        name = cf.object_name
        pos_history = self.data_history[name][:, 0:3]
        # print(pos_history[-1])
        if self.hover_flag[name] == True:
            # wps = self.hover[name]
            wps = np.array([0.0,0.0,0.5])
            # print("Hovering")
        else:
            wps = self.mover[name][0:3]
            # print("Normal flight, wps:  ",wps)
        ## update the error data
        self.error_history[name][0:data_length - 1] = self.error_history[name][1:data_length]
        self.error_history[name][-1,0:3] = wps - pos_history[-1]
        kp = 0.7
        kd = 0.0
        v_max = 0.5
        v_des = kp * self.error_history[name][-1,0:3] + kd * (
                self.error_history[name][-1,0:3] - self.error_history[name][-2,0:3]) / 0.02
        for i in range(len(v_des)):
            if v_des[i] >= v_max:
                v_des[i] = v_max
            elif v_des[i] <= -v_max:
                v_des[i] = -v_max
        cf.cf.commander.send_velocity_world_setpoint(v_des[0], v_des[1], v_des[2], 0)
        # print("cmd_vel: ",name,v_des)
        return v_des

    # @threaded
    def box_constratins(self, mover_i):
        x_max = 1
        x_min = -1
        y_max = 1
        y_min = -0.1
        z_max = 1.5
        z_min = 0.1
        if mover_i[0] >= x_max:
            mover_i[0] = x_max
        elif mover_i[0] <= x_min:
            mover_i[0] = x_min
        if mover_i[1] >= y_max:
            mover_i[1] = y_max
        elif mover_i[1] <= y_min:
            mover_i[1] = y_min
        if mover_i[2] >= z_max:
            mover_i[2] = z_max
        elif mover_i[2] <= z_min:
            mover_i[2] = z_min
        return mover_i

    ## the command is the position and yaw
    @threaded
    def ros_bridge_execute(self, Flag):
        wps_count = 0
        test_hight_cmd = 0.0
        if Flag == True:
            print("Controller Activated")
            count = 0
        while (Flag):
            t_control = time.time()
            t_elp = t_control - self.start_time
            self.time_list_control[0:9] = self.time_list_control[1:10]
            self.time_list_control[9] = float(t_control)
            dt_control = (self.time_list_control[9] - self.time_list_control[0]) / 10

            ## close the system and save
            if t_control - self.start_time >= self.duration:
                self.close()
                self.save_flag = False
                Flag = False

            # if count % self.print_t ==0:
            #     print("control dt:  ", dt_control)
            # time.sleep(0.01)
            time.sleep(0.02)
            current_pos = self.current()
            for cf in self.crazyflies:
                self.mover_history[cf.object_name][0:data_length - 1] = self.mover_history[cf.object_name][
                                                                        1:data_length]
                self.mover_history[cf.object_name][-1] = self.mover[cf.object_name]
                current_pos_i = current_pos["vicon-" + cf.object_name][1:4]
                self.data_history[cf.object_name][-1,0:3] = current_pos_i
                self.mover[cf.object_name] = self.box_constratins(self.mover[cf.object_name])
                print(self.mover[cf.object_name])
                q_i = self.mover[cf.object_name][3:7]
                euler_i = self.q2e(q_i)
                yaw_i = euler_i[-1]
                yaw_i = 0  ## set to 0 for test
                if self.mover[cf.object_name][2] == 0:  ## Hover (no cmd received or computing)
                    ## in optimization code, send zeros when computing to enable hover

                    if self.hover_flag[cf.object_name] == False:
                        self.hover[cf.object_name] = self.mover_history[cf.object_name][-2][0:3]
                        ## store the second last wps as hover point

                    self.hover_flag[cf.object_name] = True  ## enter hover mode
                    vel_des = self.vel_controller(cf)
                    self.data_history[cf.object_name][-1, 10:13] = vel_des
                    # print("vel_cmd: ",vel_des)
                    # if t_elp % 1 == 0:
                    # print("time", t_elp)
                else:
                    self.hover_flag[cf.object_name] = False
                    vel_des = self.vel_controller(cf)
                    self.data_history[cf.object_name][-1, 10:13] = vel_des
                    # print("vel_cmd: ",vel_des)
            count += 1

    def q2e(self, q: np.ndarray):
        r = rot.from_quat(q)
        e = r.as_euler("xyz", degrees=True)
        return e

    def e2q(self, e: np.ndarray):
        r = rot.from_euler("xyz", e, degrees=True)
        q = r.as_quat()
        return q

    def current(self):
        return self.curr_data

    def record(self, source, name, x, y, z, p=0, r=0, q=0, w=1):
        t = time.time()
        t_elp = t - self.t0
        line = f"{source + name},{t_elp},{x},{y},{z},{p},{r},{q},{w}"
        self.lock.acquire()
        self.log_data += (line + "\n")
        self.curr_data[source + name] = [t_elp, x, y, z, p, r, q, w]
        # if source == "vicon":
        #     self.data_history[source+name][0:-1,:] = self.data_history[name][1:data_length,:]
        #     self.data_history[source+name][-1,:] = np.array([t, x, y, z, p, r, t, w ])
        self.lock.release()

    @threaded
    def vicon(self):
        mocap = motioncapture.connect("vicon", {'hostname': self.vicon_ip})
        while self.is_open:
            t_vicon = time.time()
            self.time_list_vicon[0:9] = self.time_list_vicon[1:10]
            self.time_list_vicon[9] = float(t_vicon)
            dt = (self.time_list_vicon[9] - self.time_list_vicon[0]) / 10
            mocap.waitForNextFrame()
            pos_data = mocap.rigidBodies.items()
            for name, obj in pos_data:
                if self.record_count % self.print_t == 0:
                    self.Control_Flag = True  ## activeate the controller
                    # print("Received vicon data, dt :",dt,"state:    ",obj.position[0], obj.position[1], obj.position[2])
                self.record("vicon-", name, obj.position[0], obj.position[1], obj.position[2], obj.rotation.x,
                            obj.rotation.y, obj.rotation.z, obj.rotation.w)
                for cf in self.crazyflies:
                    if name == cf.object_name:
                        cf.cf.extpos.send_extpos(obj.position[0], obj.position[1], obj.position[2])
                        # cf.cf.extpos.send_extpose( obj.position[0], obj.position[1], obj.position[2], obj.rotation.x, obj.rotation.y, obj.rotation.z, obj.rotation.w )
            self.record_count += 1
            # if self.record_count % 50 == 0:
            #     # state = np.array([obj.position[0], obj.position[1], obj.position[2],obj.rotation.x, obj.rotation.y, obj.rotation.z, obj.rotation.w ])
            #     euler = self.q2e(np.array([obj.rotation.x, obj.rotation.y, obj.rotation.z, obj.rotation.w]))
            #     # euler[1] = - euler[1]
            #     print("Vicon euler:    ", euler)
            #     q_modi = self.e2q(euler)
            #     q_modi[2] = - q_modi[2]
            #     print("Vicon q:  ", q_modi)

    @threaded
    def logging(self):
        while self.is_open:
            self.record_count_ekf += 1
            t_ekf = time.time()
            self.time_list_ekf[0:9] = self.time_list_ekf[1:10]
            self.time_list_ekf[9] = float(t_ekf)
            dt = (self.time_list_ekf[9] - self.time_list_ekf[0]) / 10
            for cf in self.crazyflies:
                name = cf.object_name
                ## shift data
                self.data_history[name][0:data_length - 1] = self.data_history[name][1:data_length]

                current_data_i = np.zeros(10)  ## pos + att + vel
                state = np.zeros(7)
                for log_pos_entry in cf.cflog_pos:
                    pos_data = log_pos_entry[1]
                    # self.record( "kalman-" , cf.object_name, data['kalman.stateX'], data['kalman.stateY'], data['kalman.stateZ']);
                    state[0:3] = np.array([pos_data['stateEstimate.x'],
                                           pos_data['stateEstimate.y'],
                                           pos_data['stateEstimate.z']])
                    break

                for log_att_entry in cf.cflog_att:
                    att_data = log_att_entry[1]
                    state[3:7] = np.array([att_data['stateEstimate.qx'],
                                           att_data['stateEstimate.qy'],
                                           att_data['stateEstimate.qz'],
                                           att_data['stateEstimate.qw']])
                    break
                # print(state)
                for log_vel_entry in cf.cflog_vel:
                    vel_data = log_vel_entry[1]
                    vel = np.array([vel_data['stateEstimate.vx'],
                                    vel_data['stateEstimate.vy'],
                                    vel_data['stateEstimate.vz']])
                    break
                current_data_i[0:7] = state[0:7]
                current_data_i[7:10] = vel
                ## add new data (dont take pos)
                self.data_history[name][data_length - 1][3:10] = current_data_i[3:10]

                # if self.record_count_ekf % 50 == 0:
                #     print("EKF euler:   ",euler)
                #     print("EKF q:    ",state[3:7])
                self.record("kalman-", cf.object_name, state[0], state[1], state[2], state[3], state[4], state[5],
                            state[6])

    def killswitch(self):
        start_time = time.time()
        while self.is_open:
            time.sleep(1.0)
            if time.time() - start_time > self.kill_sleep:
                break
        if self.is_open:
            print(f"Shutting down crazyflies due to kill switch timeout")
            for cf in self.crazyflies:
                cf.cf.high_level_commander.stop()

    def close(self):
        if self.save_flag == True:
            print("7) Cleaning Up")
            self.is_open = False
            some_alive = True
            while some_alive:
                some_alive = False
                for thread in self.threads:
                    some_alive = some_alive or thread.is_alive()
            path = f"CF_demo_run_{round(time.time())}.csv";
            file = open(path, "w")
            file.write(self.log_data)
            file.close()
            print(f"Saved log data to: {path}")
            for cf in self.crazyflies:
                print(f"Shutting down crazyflie {cf.object_name}")
                cf.cf.high_level_commander.stop()
                cf.cf.close_link()
