#!/usr/bin/env python

import struct
import math
from math import sin, cos, pi
import threading

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3


infile_path = "/dev/odom"

# * Event types
EV_REL = 0x02

# * Relative axes
REL_X = 0x00
REL_Y = 0x01
REL_Z = 0x02
REL_RX = 0x03
REL_RY = 0x04
REL_RZ = 0x05
REL_HWHEEL = 0x06
REL_DIAL = 0x07
REL_WHEEL = 0x08
REL_MISC = 0x09
REL_MAX = 0x0f


# struct input_event {
#     struct timeval time;
#     unsigned short type;
#     unsigned short code;
#     unsigned int value;
# };

EVENT_FORMAT = "llHHi"; # long, long, unsigned short, unsigned short, signed int
EVENT_SIZE = struct.calcsize(EVENT_FORMAT)

def px2m(px):
    """ 600dpi to m """
    #return float(px) * 25.4/600.0/1000.0
    return float(px) * 0.000042333

class KameOdometer():

    def __init__(self):
        rospy.init_node('odometry_publisher')

        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
        self.odom_broadcaster = tf.TransformBroadcaster()

        self.mouse_x = 0
        self.mouse_y = 0
        self.lock = threading.Lock()

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()

        self.max_rate = rospy.get_param('~max_rate', 10.0)

    def handleMouseEvent(self):
        with open(infile_path, "rb") as file:
            event = file.read(EVENT_SIZE)
            while event and not rospy.is_shutdown():
                (tv_sec, tv_usec, type_, code, value) = struct.unpack(EVENT_FORMAT, event)
                if type_ == EV_REL:
                    if code == REL_X:
                        with self.lock:
                            self.mouse_x += value
                    elif code == REL_Y:
                        with self.lock:
                            self.mouse_y += value
                event = file.read(EVENT_SIZE)

    def handleOdometry(self):

        dt = (self.current_time - self.last_time).to_sec()

        with self.lock:
            move_x = px2m(self.mouse_x)
            move_y = 0.0
            move_th = px2m(self.mouse_y)/0.1 # r=0.1
            self.mouse_x = 0
            self.mouse_y = 0

        vx = move_x / dt
        vy = 0.0
        vth = move_th / dt

        delta_x = (move_x * cos(self.th) - move_y * sin(self.th))
        delta_y = (move_x * sin(self.th) + move_y * cos(self.th))
        delta_th = move_th

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
        odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

        # publish the message
        self.odom_pub.publish(odom)

    def spin(self):
        r = rospy.Rate(self.max_rate)
        th = threading.Thread(target=self.handleMouseEvent)
        th.start()
        while not rospy.is_shutdown():
            self.current_time = rospy.Time.now()
            self.handleOdometry()
            self.last_time = self.current_time
            r.sleep()
        #th.join()

if __name__ == '__main__':
    kame = KameOdometer()
    rospy.loginfo("=== run")
    kame.spin()
    rospy.loginfo("=== end")
