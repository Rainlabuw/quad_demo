import time
import numpy
import threading
from threading import Thread
import numpy as np
import math
from numpy import linalg as LA

import matplotlib.pyplot as plt


def threaded(fn):
    def wrapper(*a, **k):
        t = Thread(target=fn, args=a, kwargs=k, daemon=False)
        t.start();
        return t

    return wrapper


data_len = 50


class server:
    def __init__(self):
        self.cf_list = ["cf1", "cf2"]
        self.data = {}
        self.history = {}
        for cf in self.cf_list:
            self.data[cf] = np.zeros(6)
            self.history[cf] = np.zeros((data_len, 6))
        self.t0 = time.time()
        self.time_list = np.zeros(data_len)
        self.fake_gen()
        self.live_3Dplot()
        # self.live_2Dplot()
        plt.ion()
    @threaded
    def fake_gen(self):
        while (True):
            time.sleep(0.01)
            t_elp = time.time() - self.t0
            # print(t_elp)
            for cf in self.cf_list:
                if cf == "cf1":
                    self.data[cf] = np.array([np.sin(t_elp), np.cos(t_elp), np.sin(t_elp), 10 * np.cos(t_elp), 1, 2])
                else:
                    self.data[cf] = np.array([0*np.sin(t_elp), np.cos(t_elp), np.sin(t_elp), 10 * np.cos(t_elp), 1, 2])
                self.history[cf][0:data_len - 1] = self.history[cf][1:data_len]
                self.history[cf][data_len - 1, :] = self.data[cf]




    @threaded
    def live_plot(self, blit=False):

        fig = plt.figure(figsize=(5, 5))
        ax1 = fig.add_subplot(111, projection='3d')

        fig = plt.figure(figsize=(5, 5))
        ax = fig.add_subplot(2, 1, 1)

        vel_all = {}
        pos_data_all = {}
        for cf in self.cf_list:
            pos_data_all[cf],  = ax1.plot([], [], [], 'g.', lw=3)
            vel_all[cf], = ax.plot([], [], 'g.', lw=3)

        # ax1.set_xlim([-800, 1500])
        # ax1.set_ylim([-800, 1500])
        ax1.set_xlabel('X')
        ax1.set_xlabel('Y')

        fig.canvas.draw()  # note that the first draw comes before setting data

        if blit:
            # cache the background
            ax1background = fig.canvas.copy_from_bbox(ax1.bbox)

        plt.show(block=False)

        # for i in np.arange(10000):
        while (True):
            time.sleep(0.01)
            self.time_list[0:data_len - 1] = self.time_list[1:data_len]
            self.time_list[data_len - 1] = time.time() - self.t0
            print(self.time_list)
            ax.set_xlim([self.time_list[0], self.time_list[data_len-1]])
            ax.set_ylim([-10, 10])



            x_traj = {}
            y_traj = {}
            z_traj = {}
            x_vel = {}
            for cf in self.cf_list:
                x_traj[cf] = self.history[cf][:, 0]
                y_traj[cf] = self.history[cf][:, 1]
                z_traj[cf] = self.history[cf][:, 2]
                pos_data_all[cf].set_data(x_traj[cf], y_traj[cf])
                pos_data_all[cf].set_3d_properties(z_traj[cf])

                x_vel[cf] = self.history[cf][:, 0]
                vel_all[cf].set_data(self.time_list, x_vel[cf])


            # line1.set_data(x, y)
            # line1.set_3d_properties(z)

            if blit:
                # restore background
                fig.canvas.restore_region(ax1background)

                # redraw just the points
                # ax1.draw_artist(line1)

                # coords = plt.ginput(5)
                # fill in the axes rectangle
                fig.canvas.blit(ax1.bbox)


            else:

                fig.canvas.draw()

            fig.canvas.flush_events()


if __name__ == '__main__':
    Server = server()
