from ballsbot.servos import get_controls
from ballsbot.utils import keep_rps, run_as_thread


def link_controller(joystick_wrapper):
    car_controls = get_controls(scale=2.)

    def run():
        ts = None
        while True:
            ts = keep_rps(ts, fps=10)
            car_controls['steering'].run(joystick_wrapper.steering)
            car_controls['throttle'].run(joystick_wrapper.throttle, joystick_wrapper.turbo_pressed)

    run_as_thread(run)
