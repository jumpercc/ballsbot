from time import time, sleep
import threading
import atexit


def keep_rps(ts, fps=1.):
    new_ts = time()
    if ts is not None:
        to_sleep = ts - new_ts + 1. / fps
        if to_sleep > 0:
            sleep(to_sleep)
    return new_ts


def run_as_thread(an_action):
    a_thread = threading.Thread(target=an_action)

    def stop_me():
        a_thread.join()

    a_thread.start()
    atexit.register(stop_me)
