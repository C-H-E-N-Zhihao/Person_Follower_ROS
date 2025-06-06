import rospy
import math
from sensor_msgs.msg import LaserScan

class ObstacleDetector:
    def __init__(self):
        self.obstacle_front = False
        self.laser_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        self.latest_scan = None
       
        self.front_indices = None
        self.left_indices = None

    def precalculate_indices(self, scan_msg):
        """Pre-calculate and store the indices for front and left sectors."""
        self.front_indices = (
            self.angle_to_index(345, scan_msg),
            self.angle_to_index(359, scan_msg),
            self.angle_to_index(0, scan_msg),
            self.angle_to_index(15, scan_msg)
        )
        self.left_indices = (
            self.angle_to_index(75, scan_msg),
            self.angle_to_index(105, scan_msg)
        )

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

    def check_obstacle_in_sector(self, scan_msg, start_idx, end_idx, min_distance=1):
        """Check if there is an obstacle within min_distance in the angle sector."""
        for i in range(start_idx, end_idx + 1):
            dist = scan_msg.ranges[i]
            if dist > 0 and dist < min_distance:
                return True
        return False

    def scan_callback(self, msg):
        """Store latest LaserScan data."""
        self.latest_scan = msg
        if self.latest_scan:
            self.precalculate_indices(self.latest_scan)

    def is_obstacle_front(self):
        if self.latest_scan is None:
            return False
        else:
            start_idx_1, end_idx_1, start_idx_2, end_idx_2 = self.front_indices
            return self.check_obstacle_in_sector(self.latest_scan, start_idx_2, end_idx_2, 1) or self.check_obstacle_in_sector(self.latest_scan, start_idx_1, end_idx_1, 1)

    def is_obstacle_left(self):
        if self.latest_scan is None:
            return False
        else:
            start_idx, end_idx = self.left_indices
            return self.check_obstacle_in_sector(self.latest_scan, start_idx, end_idx, 0.15)
