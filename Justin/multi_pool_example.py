import time
import numpy as np
import multiprocessing as mp
from single_scvx_calss import optmization_template

def traj_opt_pair(args):
    x_ini_0, x_des_0 = args
    return optmization_template(x_ini_0, x_des_0).X_traj["robot01"][-1]

if __name__ == "__main__":
    pool = mp.Pool(processes=3)   # two worker processes
    futures = [None, None,None]        # slots for up to 2 pending jobs

    t0 = time.time()

    while True:
        time.sleep(0.01)
        t_esp = time.time() - t0

        x_des_0 = np.array([1.5 + np.sin(t_esp)*0.1, 1.0, 1.0, 0,0,0, 0,0,0])
        x_ini_0 = np.array([np.sin(t_esp)*0.1,       1.0, 1.0, 0,0,0, 0,0,0])

        #  submit new jobs in any free slot
        for i in range(3):
            if futures[i] is None or futures[i].ready():
                futures[i] = pool.apply_async(traj_opt_pair,
                                                  args=((x_ini_0, x_des_0),))

        #  collect any finished results
        for i, fut in enumerate(futures):
            if fut is not None and fut.ready():
                sol = fut.get()
                print(f"[worker {i}] result:", sol)
                # mark slot free so you can launch a new one next iteration
                futures[i] = None



