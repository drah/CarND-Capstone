import rospy
import pid

ONE_MPH = 0.44704

class ThrottleBrakeController(object):
    def __init__(self, kp, ki, kd, max_speed, accel_limit, decel_limit, brake_coef, **kwargs):
        self.pid = pid.PID(kp, ki, kd)
        self.max_speed = max_speed
        self.throttle_min = kwargs.get('throttle_min', 0.)
        self.throttle_max = kwargs.get('throttle_max', 0.3)
        self.accel_limit = accel_limit
        self.decel_limit = decel_limit
        self.brake_coef = brake_coef

    def get_throttle_brake(
            self,
            target_linear_velocity,
            target_angular_velocity,
            current_linear_velocity,
            delta_t):

        # max_delta_v = self.accel_limit * delta_t
        # min_delta_v = self.decel_limit * delta_t

        target_linear_velocity = min(target_linear_velocity, self.max_speed)
        error = target_linear_velocity - current_linear_velocity
        # error = min(error, max_delta_v)
        # error = max(error, min_delta_v)

        throttle = self.pid.step(error, delta_t)
        throttle = min(self.throttle_max, throttle)
        throttle = max(self.throttle_min, throttle)

        if target_linear_velocity < 0.1 and current_linear_velocity < 0.1:
            brake = 800
        else:
            brake = abs(error) * self.brake_coef

        if error >= 0: # acc
            brake = 0.
        else: # dec
            throttle = 0.

        return throttle, brake
