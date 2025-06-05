#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import LaserScan

class ObstacleDetector:
    def __init__(self):
        self.obstacle_front = False
        self.min_distance = 1 # threshold distance in meters
        self.laser_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback)

    def angle_to_index(self, angle_deg, scan_msg):
        """Convert angle in degrees to index in ranges array."""
        angle_rad = math.radians(angle_deg)
        index = int((angle_rad - scan_msg.angle_min) / scan_msg.angle_increment)
        # Clamp index to valid range
        if index < 0:
            index = 0
        elif index >= len(scan_msg.ranges):
            index = len(scan_msg.ranges) - 1
        return index

    def check_obstacle_in_sector(self, scan_msg, start_deg, end_deg):
        """Check if there is an obstacle within min_distance in angle sector."""
        start_idx = self.angle_to_index(start_deg, scan_msg)
        end_idx = self.angle_to_index(end_deg, scan_msg)
        for i in range(start_idx, end_idx + 1):
            dist = scan_msg.ranges[i]
            if dist > 0 and dist < self.min_distance:
                return True
        return False

    def scan_callback(self, msg):
        # Check obstacle in front sector (around ±20 degrees)
        self.obstacle_front = self.check_obstacle_in_sector(msg, 340, 359) or \
                   self.check_obstacle_in_sector(msg, 0, 20)

        # Check obstacle in left sector (around ±20 degrees)
        self.obstacle_left = self.check_obstacle_in_sector(msg, 70, 110)

    def is_obstacle_front(self):
        return self.obstacle_front

    def is_obstacle_left(self):
        return self.obstacle_left