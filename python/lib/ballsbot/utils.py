from time import time, sleep
import threading
import atexit
import cv2
# import inspect


class PropagatingThread(threading.Thread):
    def run(self):
        self.exc = None  # pylint: disable=W0201
        try:
            if self._target:
                self._target(*self._args, **self._kwargs)
        except BaseException as e:  # pylint: disable=W0703
            self.exc = e  # pylint: disable=W0201
        finally:
            del self._target, self._args, self._kwargs

    def join(self, timeout=None):
        super().join(timeout)
        if self.exc:
            raise self.exc


def keep_rps(ts, fps=1.):  # pylint: disable=C0103
    new_ts = time()
    if ts is not None:
        to_sleep = ts - new_ts + 1. / fps
        if to_sleep > 0:
            # print('{} slipping for {:0.03f}'.format(inspect.stack()[1][3], to_sleep))
            sleep(to_sleep)
        # else:
        #     print(
        #         '{} no slipping ({} - {} + {} = {} < 0)'.format(
        #             inspect.stack()[1][3], ts, new_ts, 1./fps, to_sleep
        #         )
        #     )
    return new_ts


_all_threads = []


def run_as_thread(an_action, stop_action=None, *args, **kwargs):  # pylint: disable=W1113
    a_thread = PropagatingThread(target=an_action, args=args, kwargs=kwargs)

    def stop_me():
        a_thread.join()
        if stop_action:
            stop_action()

    a_thread.start()
    atexit.register(stop_me)
    _all_threads.append(a_thread)


def join_all_threads():
    my_ident = threading.current_thread().ident
    for a_thread in _all_threads:
        if my_ident != a_thread.ident:
            a_thread.join(timeout=0.1)


def bgr8_to_jpeg(value):
    return bytes(cv2.imencode('.jpg', value)[1])  # pylint: disable=E1101


def figsize_from_image_size(image):
    return int(image.width) / 100., int(image.height) / 100.
