#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan

class UltrasonicScanner():
    def __init__(self):
        rospy.init_node('laser_scan_publisher')

        self.scan_pub = rospy.Publisher('scan', LaserScan, queue_size=50)
        self.num_readings = 180
        self.laser_frequency = 40

        self.rate = 1.0

    def handleScanner(self):
        scan = LaserScan()

        scan.header.stamp = self.current_time
        scan.header.frame_id = 'laser_frame'
        scan.angle_min = -1.57
        scan.angle_max = 1.57
        scan.angle_increment = 3.14 / self.num_readings
        scan.time_increment = 0.003012 * self.num_readings
        scan.range_min = 0.02  # [m]
        scan.range_max = 4.00  # [m]

        scan.ranges = []
        scan.intensities = []
        for i in range(0, self.num_readings):
            scan.ranges.append(3.0) # fake data
            scan.intensities.append(1) # fake data

        self.scan_pub.publish(scan)

    def spin(self):
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.current_time = rospy.Time.now()
            self.handleScanner()
            r.sleep()

if __name__ == '__main__':
    scanner = UltrasonicScanner()
    rospy.loginfo("=== run")
    scanner.spin()
    rospy.loginfo("=== end")
