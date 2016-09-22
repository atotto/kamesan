#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from pololu_drv8835_rpi import motors, MAX_SPEED


class KameDriver():

    def __init__(self):
        rospy.init_node('kame_driver')

        motors.setSpeeds(0, 0)

        self.radius = rospy.get_param("~base_width", 0.129) / 2
        max_speed = rospy.get_param("~max_speed", 0.398)
        self.speed_ratio = 1.0 * MAX_SPEED / max_speed

        sub = rospy.Subscriber('/cmd_vel', Twist, self.handleMotor)

    def handleMotor(self, msg):
        dx = msg.linear.x   # [m/s]
        dr = msg.angular.z  # [rad/s]

        rv = int((1.0 * dx + dr * self.radius) * self.speed_ratio)
        lv = int((1.0 * dx - dr * self.radius) * self.speed_ratio)

        rospy.loginfo("drive: in(x:%+02.2f r:%+02.2f) out:(L:%4d R:%4d)", dx, dr, lv, rv)
        motors.setSpeeds(int(rv), int(lv))

    def spin(self):
        rospy.spin()

if __name__ == '__main__':
    driver = KameDriver()
    rospy.loginfo("=== run")
    driver.spin()
