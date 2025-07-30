import numpy as np

import threading
import time
from threading import Thread, Event
import multiprocessing as mp
from .live_plotting import plotting_server
from .single_scvx_calss import optmization_template


def traj_opt_pair(args):
    x_ini_0, x_des_0 = args
    return optmization_template(x_ini_0, x_des_0).X_traj["robot01"]


def threaded(fn):
    def wrapper(*args, **kwargs):
        thread = Thread(target=fn, args=args, kwargs=kwargs)
        thread.start()
        return thread

    return wrapper


class client_main:
    def __init__(self, cf_list):
        self.plotting = plotting_server(cf_list)
        self.pool = mp.Pool(processes=2)  ## spawn two workers
        self.cf_list = cf_list
        self.cf_mover = {}
        self.state = {}
        self.target = {}
        self.target_buffer = {}
        self.state_history = {}
        self.data_length = 20
        self.traj_all = {}
        self.traj_T = 51
        self.traj_flag = {}
        self.send_count = 0
        self.traj_t0 = 0.0
        self.traj_time_series = np.linspace(0, 5, self.traj_T)
        self.manager = mp.Manager()
        self.state_queue = self.manager.Queue()
        self.traj_queue = self.manager.Queue()
        self.target_queue = self.manager.Queue()


        self.listen_target()

        self.init()
        time.sleep(0.1)
        self.t0 = time.time()
        # self.fake_gen()

        self.update()
        self.send_data()

    def init(self):
        for cf in self.cf_list:
            self.cf_mover[cf] = np.zeros(3)
            self.state[cf] = np.zeros(3)
            self.traj_all[cf] = np.zeros((self.traj_T, 3))
            self.state_history[cf] = np.zeros((self.data_length, 3))
            self.traj_flag[cf] = False

    # @threaded
    # def fake_gen(self):
    #     while True:
    #         time.sleep(0.01)
    #         t_elp = time.time() - self.t0
    #         for cf in self.cf_list:
    #             self.state[cf] = np.array([np.sin(t_elp) * 0.1, 1.0+np.cos(t_elp)*0.1, 0.7])


    @threaded
    def update(self):
        while True:
            time.sleep(0.02)
            for cf in self.cf_list:
                self.state_history[cf][0:self.data_length - 1] = self.state_history[cf][1:self.data_length]
                self.state_history[cf][-1] = self.state[cf]
                self.plotting.data[cf] = np.hstack((self.state[cf], np.zeros(3)))
                # print("client",self.state[cf][0:3])
            # push your current state to the plot‐process
            # make a shallow‐copied dict so pickling is safe
            # print(self.state_queue)

    @threaded
    def listen_target(self):
        while True:
            time.sleep(0.1)
            self.target = self.plotting.target_click
            if self.target != {} and self.plotting.opt_flag == True:
                print("clicked")
                for cf in self.cf_list:
                    self.target[cf][2] = 0.5
                print("Client received targets:", self.target)
                self.opt()

    def opt(self):
        for cf in self.cf_list:
            self.traj_flag[cf] = False
        print("opt being, enter hovering")
        x_ini_all = self.state.copy()
        x_des_all = self.target.copy()

        ## single cf test
        x_ini_0 = x_ini_all[self.cf_list[0]]
        x_ini_0 = np.hstack((x_ini_0))
        x_des_0 = x_des_all[self.cf_list[0]]
        # run async
        sol = self.pool.apply_async(traj_opt_pair, args=((x_ini_0, x_des_0),))
        # get result
        self.traj_all[self.cf_list[0]] = sol.get()  # blocks until done
        # time.sleep(1.0)
        self.plotting.traj_all = self.traj_all
        for cf in self.cf_list:
            self.traj_flag[cf] = True
        self.traj_t0 = time.time()
        self.plotting.opt_flag = False
        print("opt finished, execute")

    @threaded
    def send_data(self):
        while True:
            time.sleep(0.02)
            self.send_count += 1
            for cf in self.cf_list:
                if self.traj_flag[cf] == True:
                    traj_time = time.time() - self.traj_t0
                    wps_idx = np.argmin(np.abs(self.traj_time_series - traj_time))  ## current wps index
                    if wps_idx > self.traj_T - 1:
                        wps_idx = self.traj_T - 1
                    self.cf_mover[cf] = self.traj_all[cf][wps_idx]
                    if self.send_count % 10 == 0:
                        print("Cmd pos to:", np.round(self.cf_mover[cf], 2))
                else:
                    ## hover
                    if self.target != {}:
                        self.cf_mover[cf] = self.target[cf]
                    else:
                        self.cf_mover[cf] = np.array([0.,0.,0.5])
                    if self.send_count % 10 == 0:
                        print("Hovering cmd to:", self.cf_mover[cf])


# if __name__ == '__main__':
#     cf_list = ["cf01"]
#     cf = client_main(cf_list)
