from time import time, sleep
import threading
import atexit
import cv2


# import inspect


def keep_rps(ts, fps=1.):
    new_ts = time()
    if ts is not None:
        to_sleep = ts - new_ts + 1. / fps
        if to_sleep > 0:
            # print('{} slipping for {:0.03f}'.format(inspect.stack()[1][3], to_sleep))
            sleep(to_sleep)
        # else:
        #     print('{} no slipping ({} - {} + {} = {} < 0)'.format(inspect.stack()[1][3], ts, new_ts, 1./fps, to_sleep))
    return new_ts


def run_as_thread(an_action):
    a_thread = threading.Thread(target=an_action)

    def stop_me():
        a_thread.join()

    a_thread.start()
    atexit.register(stop_me)


def bgr8_to_jpeg(value):
    return bytes(cv2.imencode('.jpg', value)[1])
