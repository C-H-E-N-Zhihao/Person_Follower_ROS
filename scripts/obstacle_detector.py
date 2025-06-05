#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import LaserScan

class ObstacleDetector:
    def __init__(self):
        self.obstacle_front = False
        self.laser_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        self.latest_scan = None
        self.indices_ready = False

    def scan_callback(self, msg):
        """Callback to store latest LaserScan and precalculate angle indices once."""
        self.latest_scan = msg
        if not self.indices_ready:
            self.front_indices = self.get_indices_for_sector(msg, [(340, 359), (0, 20)])
            self.left_indices = self.get_indices_for_sector(msg, [(70, 110)])
            self.indices_ready = True

    def get_indices_for_sector(self, scan_msg, angle_ranges):
        """Convert angle sectors to list of index ranges in the scan array."""
        indices = []
        for start_deg, end_deg in angle_ranges:
            start_idx = self.angle_to_index(start_deg, scan_msg)
            end_idx = self.angle_to_index(end_deg, scan_msg)
            indices.extend(range(start_idx, end_idx + 1))
        return indices

    def angle_to_index(self, angle_deg, scan_msg):
        """Convert a given angle in degrees to corresponding index in ranges array."""
        angle_rad = math.radians(angle_deg)
        index = int((angle_rad - scan_msg.angle_min) / scan_msg.angle_increment)
        return max(0, min(index, len(scan_msg.ranges) - 1))

    def check_obstacle_in_indices(self, scan_msg, indices, min_distance):
        """Check if any distance reading in given indices is below the minimum threshold."""
        for i in indices:
            dist = scan_msg.ranges[i]
            if dist > 0 and dist < min_distance:
                return True
        return False

    def is_obstacle_front(self):
        """Return True if an obstacle is detected in front sector."""
        if self.latest_scan is None or not self.indices_ready:
            return False
        return self.check_obstacle_in_indices(self.latest_scan, self.front_indices, 0.75)

    def is_obstacle_left(self):
        """Return True if an obstacle is detected in left sector."""
        if self.latest_scan is None or not self.indices_ready:
            return False
        return self.check_obstacle_in_indices(self.latest_scan, self.left_indices, 0.15)