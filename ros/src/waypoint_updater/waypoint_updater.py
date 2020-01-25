#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from scipy.spatial import KDTree

import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        # rospy.Subscriber('/traffic_waypoint',
        # rospy.Subscriber('/obstacle_waypoint',

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.pose = None
        self.base_waypoints = None
        self.waypoints_2d = None
        self.waypoint_tree = None
        self.n_ahead_waypoints = LOOKAHEAD_WPS

        self.loop()

    def loop(self):
        rate = rospy.Rate(50) # greater than 30 is OK
        while not rospy.is_shutdown():
            if self.pose and self.base_waypoints:
                self.publish_waypoints()
            rate.sleep()

    def prepare_and_publish_waypoints(self):
        waypoints = WaypointUpdater.get_waypoints_ahead(
                self.pose,
                self.base_waypoints,
                self.waypoints_2d,
                self.waypoint_tree,
                self.n_ahead_waypoints)
        self.publish_waypoints(waypoints)

    @staticmethod
    def get_waypoints_ahead(
            car_pose, #: PoseStamped,
            base_waypoints, #: [Waypoint],
            waypoints_2d, #: [[float, float]],
            waypoint_tree, #: KDTree,
            n_ahead_waypoints): #: int) -> [Waypoint]:
        car_x = car_pose.pose.pose.position.x
        car_y = car_pose.pose.pose.position.y
        index_of_closest_wp = waypoint_tree.query([car_x, car_y], 1)[1]
        closest_xy = waypoints_2d[index_of_closest_wp]
        before_closest_xy = waypoints_2d[index_of_closest_wp - 1]

        dot_value = WaypointUpdater.get_dot([car_x, car_y], closest_xy, before_closest_xy)
        if dot_value > 0:
            index_of_closest_wp = (index_of_closest_wp + 1) % len(waypoints_2d)
        return base_waypoints[index_of_closest_wp:(index_of_closest_wp + n_ahead_waypoints)]

    @staticmethod
    def get_dot(
            p, #: [float, float],
            p1, #: [float, float],
            p2): #: [float, float]):
        return (p1[0] - p[0]) * (p2[0] - p[0]) + (p1[1] - p[1]) * (p2[1] - p[1])

    def publish_waypoints(
            self,
            waypoints): #: [Waypoint]):
        lane = Lane()
        lane.header = self.base_waypoints.header
        lane.waypoints = waypoints
        self.final_waypoints_pub.publish(lane)

    def pose_cb(
            self,
            msg): #: PoseStamped):
        self.pose = msg

    def waypoints_cb(
            self,
            waypoints): #: Lane):
        self.base_waypoints = waypoints
        if not self.waypoints_2d:
            self.waypoints_2d = [[wp.pose.pose.position.x, wp.pose.pose.position.y] for wp in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
