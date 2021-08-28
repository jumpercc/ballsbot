from ballsbot.servos import get_controls
from ballsbot.utils import keep_rps, run_as_thread
from ballsbot.config import CAR_CONTROLS


def link_controller(controller):
    car_controls = get_controls()

    steering_config = CAR_CONTROLS['steering']['control']
    throttle_config = CAR_CONTROLS['throttle']['control']
    throttle_reverse = throttle_config.get('reverse')

    def update_steering(value):
        car_controls['steering'].run(value['new'])

    def update_throttle(value):
        car_controls['throttle'].run(-value['new'] if throttle_reverse else value['new'])

    def link_controls():
        ts = None
        while True:
            ts = keep_rps(ts, fps=1)

            if len(controller.axes):
                controller.axes[steering_config['axis']].observe(update_steering, 'value')
                controller.axes[throttle_config['axis']].observe(update_throttle, 'value')
                break

    run_as_thread(link_controls)
