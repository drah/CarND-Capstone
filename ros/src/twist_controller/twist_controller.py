import pid
import lowpass
import yaw_controller


GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(
            self, 
            wheel_base,
            steer_ratio,
            min_speed,
            max_lat_accel,
            max_steer_angle,
            **kwargs):

        min_speed_mph = min_speed * ONE_MPH
        self.yaw_controller = yaw_controller.YawController(
                wheel_base=wheel_base,
                steer_ratio=steer_ratio,
                min_speed=min_speed_mph,
                max_lat_accel=max_lat_accel,
                max_steer_angle=max_steer_angle)
        self.lowpass_filter = lowpass.LowPassFilter(kwargs.get('tau', 0.2),kwargs.get('ts', 0.1))

    def control(
            self,
            linear_velocity,
            angular_velocity,
            current_velocity,
            *args,
            **kwargs):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        steer = self.yaw_controller.get_steering(
                linear_velocity=linear_velocity,
                angular_velocity=angular_velocity,
                current_velocity=current_velocity)
        steer = self.lowpass_filter.filt(steer)

        return 1., 0., steer
