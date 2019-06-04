#!/usr/bin/env python

import rospy

from collision_checker.srv import Check, CheckRequest
from collision_checker.srv import SetConfig, SetConfigRequest
from geometry_msgs.msg import Point32
from geometry_msgs.msg import Polygon
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray

import math


class Demo(object):
    COLOR_GREEN = '#30FF79'
    COLOR_RED = '#FF4747'

    def __init__(self):
        # Get ROS parameters
        self.robot_ = Polygon()
        for vertex in rospy.get_param('~robot'):
            p = Point32()
            p.x = vertex[0]
            p.y = vertex[1]
            self.robot_.points.append(p)
        self.allow_unknown_ = rospy.get_param('~allow_unknown', True)
        res_deg = rospy.get_param('~theta_resolution_deg', 1.0)
        self.theta_resolution_ = res_deg * math.pi / 180
        self.occupancy_threshold_ = rospy.get_param('~occupancy_threshold', 10)

        self.set_config()

        self.service_ = rospy.ServiceProxy('collision_check', Check)
        self.pub_footprint_ = rospy.Publisher('footprint',
                                              OccupancyGrid,
                                              queue_size=1)
        self.pub_robot_ = rospy.Publisher('robot',
                                          MarkerArray,
                                          queue_size=1)

        rospy.Subscriber('initialpose',
                         PoseWithCovarianceStamped,
                         self.cb_pose,
                         queue_size=1)

    def set_config(self):
        req = SetConfigRequest()
        req.robot = self.robot_
        req.allow_unknown = self.allow_unknown_
        req.theta_resolution = self.theta_resolution_
        req.occupancy_threshold = self.occupancy_threshold_

        rospy.wait_for_service('set_config')
        service_config = rospy.ServiceProxy('set_config', SetConfig)
        service_config.call(req)

    def cb_pose(self, pwc):
        """
        Callback function for Rviz interactive pose marker

        :param pwc:
        :type pwc: PoseWithCovarianceStamped
        :return:
        """
        pose = pwc.pose.pose
        ps = PoseStamped()
        ps.header = pwc.header
        ps.pose = pose

        req = CheckRequest()
        req.pose = pose
        res = self.service_.call(req)

        self.pub_footprint_.publish(res.footprint)
        if res.is_valid:
            color = Demo.COLOR_GREEN
        else:
            color = Demo.COLOR_RED
        robot = Demo.make_polygon_marker(ps, self.robot_, color)
        self.pub_robot_.publish(robot)

    @staticmethod
    def make_polygon_marker(ps, polygon, color=COLOR_GREEN):
        """
        Creates a Marker of points that represent the given polygon, offset at
        the given PoseStamped. The assumption is that the first and last
        points in the polygon are connected
        :param ps:
        :type ps: PoseStamped
        :param polygon:
        :type polygon: Polygon
        :param color:
        :type color: str
        :return:
        """
        m = Marker()
        m.header = ps.header
        m.id = 0
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = 0.1
        m.points = polygon.points
        # Connect first and last points
        m.points.append(polygon.points[0])
        m.pose = ps.pose
        rgb = Demo.hex_to_rgb(color)
        m.color.r = rgb[0]
        m.color.g = rgb[1]
        m.color.b = rgb[2]
        m.color.a = 1

        ma = MarkerArray()
        ma.markers.append(m)
        return ma

    @staticmethod
    def hex_to_rgb(hex_code):
        """
        Converts a hex (#5582FF) to RGB in [0, 1] (e.g. (0.33, 0.51, 1.0)
        :param hex_code:
        :type hex_code: str
        :return:
        """
        s = hex_code.lstrip('#')
        return [int(s[i:i+2], 16) / 255.0 for i in (0, 2, 4)]


def main():
    rospy.init_node('collision_checker_demo_node')

    demo = Demo()

    rospy.spin()


if __name__ == '__main__':
    main()
