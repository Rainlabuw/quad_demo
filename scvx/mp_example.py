import time
import numpy as np
import multiprocessing as mp
from single_scvx_calss import optmization_template

def traj_opt_pair(args):
    x_ini_0, x_des_0 = args
    return optmization_template(x_ini_0, x_des_0).X_traj["robot01"][-1]

if __name__ == '__main__':
    pool = mp.Pool(processes=2)   # e.g. 2 parallel optimizers
    future = None

    t0 = time.time()
    sol = None
    try:
        while True:
            time.sleep(0.01)
            t_esp = time.time() - t0
            print(t_esp)
            x_des_0 = np.array([1.5 + np.sin(t_esp)*0.1, 1.0, 1.0, 0,0,0, 0,0,0])
            x_ini_0 = np.array([np.sin(t_esp)*0.1,       1.0, 1.0, 0,0,0, 0,0,0])

            # only submit a new job if the previous one finished
            if future is None or future.ready():
                future = pool.apply_async(traj_opt_pair, args=((x_ini_0, x_des_0),))

            # poll your result non‑blockingly
            if future is not None and future.ready():
                sol = future.get()
                print("got new sol:", sol)
                future = None  # allow re‑submission

    except KeyboardInterrupt:
        pass
    finally:
        pool.close()
        pool.join()
