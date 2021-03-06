#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
import math

from twist_controller import Controller

'''
You can build this node only after you have built (or partially built) the `waypoint_updater` node.

You will subscribe to `/twist_cmd` message which provides the proposed linear and angular velocities.
You can subscribe to any other message that you find important or refer to the document for list
of messages subscribed to by the reference implementation of this node.

One thing to keep in mind while building this node and the `twist_controller` class is the status
of `dbw_enabled`. While in the simulator, its enabled all the time, in the real car, that will
not be the case. This may cause your PID controller to accumulate error because the car could
temporarily be driven by a human instead of your controller.

We have provided two launch files with this node. Vehicle specific values (like vehicle_mass,
wheel_base) etc should not be altered in these files.

We have also provided some reference implementations for PID controller and other utility classes.
You are free to use them or build your own.

Once you have the proposed throttle, brake, and steer values, publish it on the various publishers
that we have created in the `__init__` function.

'''

class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node')

        vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        brake_deadband = rospy.get_param('~brake_deadband', .1)
        decel_limit = rospy.get_param('~decel_limit', -5)
        accel_limit = rospy.get_param('~accel_limit', 1.)
        wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        wheel_base = rospy.get_param('~wheel_base', 2.8498)
        steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        max_steer_angle = rospy.get_param('~max_steer_angle', 8.)

        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)

        self.min_speed = 1.0
        self.max_speed = 40.
        self.controller = Controller(
                throttle_kp=0.3,
                throttle_ki=0.1,
                throttle_kd=0.0,
                max_speed=self.max_speed,
                accel_limit=accel_limit,
                decel_limit=decel_limit,
                wheel_base=wheel_base,
                steer_ratio=steer_ratio,
                min_speed=self.min_speed,
                max_lat_accel=max_lat_accel,
                max_steer_angle=max_steer_angle,
                vehicle_mass=vehicle_mass,
                wheel_radius=wheel_radius)

        self.dbw_enabled_sub = rospy.Subscriber('/dbw_enabled', Bool, self.dbw_enabled_cb)
        self.cur_velocity_sub = rospy.Subscriber('/current_velocity', TwistStamped, self.cur_velocity_cb)
        self.twist_cmd_sub = rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cmd_cb)

        self.current_velocity = None
        self.linear_velocity = None
        self.angular_velocity = None
        self.dbw_enabled = True

        self.loop()

    def loop(self):
        rate = rospy.Rate(50) # 50Hz, important, >= 50 is fine, 50~20 is not good, < 20 is not okay.
        while not rospy.is_shutdown():
            if self.dbw_enabled and \
                    self.current_velocity is not None and \
                    self.linear_velocity is not None and \
                    self.angular_velocity is not None:
                throttle, brake, steer = self.controller.control(
                        linear_velocity=self.linear_velocity,
                        angular_velocity=self.angular_velocity,
                        current_velocity=self.current_velocity,
                        dbw_enabled=self.dbw_enabled)
                self.publish(throttle, brake, steer)
            else:
                rospy.loginfo("Not publish control")
            rate.sleep()

    def publish(self, throttle, brake, steer):
        # brake # in N*m, 700 for stop

        if brake != 0:
            throttle = 0

        tcmd = ThrottleCmd()
        tcmd.enable = True
        tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        tcmd.pedal_cmd = throttle
        self.throttle_pub.publish(tcmd)

        bcmd = BrakeCmd()
        bcmd.enable = True
        bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
        bcmd.pedal_cmd = brake
        self.brake_pub.publish(bcmd)

        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        self.steer_pub.publish(scmd)


    def dbw_enabled_cb(
            self,
            dbw_enabled_msg): # Bool, from /vehicle/dbw_enabled
        self.dbw_enabled = dbw_enabled_msg.data
        # rospy.loginfo("DBW enabled: %s" % self.dbw_enabled)

    def cur_velocity_cb(
            self,
            twist_stamp_msg): # TwistStamped from /current_velocity
        self.current_velocity = twist_stamp_msg.twist.linear.x

    def twist_cmd_cb(
            self,
            twist_stamp_msg): # TwistStamped from /twist_cmd
        self.linear_velocity = twist_stamp_msg.twist.linear.x
        self.angular_velocity = twist_stamp_msg.twist.angular.z




if __name__ == '__main__':
    DBWNode()
