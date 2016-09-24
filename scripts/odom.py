#!/usr/bin/env python

# refs http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom

import math
from math import sin, cos, pi

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3


class KameOdometer():

    def __init__(self):
        rospy.init_node('odometry_publisher')

        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
        self.odom_broadcaster = tf.TransformBroadcaster()

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        self.vx = 0.1
        self.vy = -0.1
        self.vth = 0.1

        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()

        self.max_rate = rospy.get_param('~max_rate', 30.0)

    def handleOdometry(self):

        dt = (self.current_time - self.last_time).to_sec()

        delta_x = (self.vx * cos(self.th) - self.vy * sin(self.th)) * dt
        delta_y = (self.vx * sin(self.th) + self.vy * cos(self.th)) * dt
        delta_th = self.vth * dt

        self.x += delta_x
        self.y += delta_y
        self.th += delta_th

        odom_quat = (0., 0., sin(self.th / 2.), cos(self.th / 2.))

        # send the transform
        self.odom_broadcaster.sendTransform(
            (self.x, self.y, 0.),
            odom_quat,
            self.current_time,
            "base_link",
            "odom"
        )

        odom = Odometry()
        odom.header.stamp = self.current_time
        odom.header.frame_id = "odom"

        odom.pose.pose = Pose(Point(self.x, self.y, 0.), Quaternion(*odom_quat))

        odom.child_frame_id = "base_link"
        odom.twist.twist = Twist(Vector3(self.vx, self.vy, 0), Vector3(0, 0, self.vth))

        # publish the message
        self.odom_pub.publish(odom)

    def spin(self):
        r = rospy.Rate(self.max_rate)
        while not rospy.is_shutdown():
            self.current_time = rospy.Time.now()
            self.handleOdometry()
            self.last_time = self.current_time
            r.sleep()

if __name__ == '__main__':
    kame = KameOdometer()
    rospy.loginfo("=== run")
    kame.spin()
    rospy.loginfo("=== end")
