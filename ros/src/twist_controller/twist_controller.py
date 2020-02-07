import time

import pid
import lowpass
import yaw_controller
import throttle_brake_controller


GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(
            self, 
            throttle_kp,
            throttle_ki,
            throttle_kd,
            max_speed,
            accel_limit,
            decel_limit,
            wheel_base,
            steer_ratio,
            min_speed,
            max_lat_accel,
            max_steer_angle,
            vehicle_mass,
            wheel_radius,
            **kwargs):

        self.throttle_brake_controller = throttle_brake_controller.ThrottleBrakeController(
                kp=throttle_kp,
                ki=throttle_ki,
                kd=throttle_kd,
                max_speed=max_speed,
                accel_limit=accel_limit,
                decel_limit=decel_limit,
                brake_coef=vehicle_mass * wheel_radius)

        self.yaw_controller = yaw_controller.YawController(
                wheel_base=wheel_base,
                steer_ratio=steer_ratio,
                min_speed=min_speed,
                max_lat_accel=max_lat_accel,
                max_steer_angle=max_steer_angle)
        self.lowpass_filter = lowpass.LowPassFilter(kwargs.get('tau', 0.2),kwargs.get('ts', 0.1))

        self.last_time = None


    def control(
            self,
            linear_velocity,
            angular_velocity,
            current_velocity,
            dbw_enabled,
            **kwargs):
        if self.last_time is None or not dbw_enabled:
            throttle, brake, steer = 0., 0., 0.

        else:
            delta_t = time.time() - self.last_time

            throttle, brake = self.throttle_brake_controller.get_throttle_brake(
                    target_linear_velocity=linear_velocity,
                    target_angular_velocity=angular_velocity,
                    current_linear_velocity=current_velocity,
                    delta_t=delta_t)

            steer = self.yaw_controller.get_steering(
                    linear_velocity=linear_velocity,
                    angular_velocity=angular_velocity,
                    current_velocity=current_velocity)
            # steer = self.lowpass_filter.filt(steer)

        self.last_time = time.time()

        return throttle, brake, steer
