import ros.src.twist_controller.pid as pid
import ros.src.twist_controller.lowpass as lowpass
import ros.src.twist_controller.yaw_controller as yaw_controller


GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement
        pass

    def control(self, *args, **kwargs):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        return 1., 0., 0.
