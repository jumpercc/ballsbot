from ballsbot.servos import get_controls
from ballsbot.utils import keep_rps, run_as_thread

running = False  # pylint: disable=C0103


def link_controller(joystick_wrapper):
    car_controls = get_controls(scale=2.)

    def run():
        global running  # pylint: disable=W0603, C0103
        running = True
        ts = None
        while running:
            ts = keep_rps(ts, fps=10)
            car_controls['steering'].run(joystick_wrapper.steering)
            car_controls['throttle'].run(joystick_wrapper.throttle, joystick_wrapper.turbo_pressed)

    run_as_thread(run)


def stop_controller():
    global running  # pylint: disable=W0603, C0103
    running = False
