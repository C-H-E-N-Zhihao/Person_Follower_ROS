#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import math
from odom_helper import OdomHelper
from move_robot import MoveKobuki

class RobotController:
    # Constants
    CENTER_WAITING_PERSON = (320, 177)
    STATES = {
        'SEARCHING': 'SEARCHING',
        'FOLLOWING': 'FOLLOWING', 
        'AVOIDING': 'AVOIDING',
        'WAITING': 'WAITING'
    }
    
    def __init__(self):
        self.move_kobuki = MoveKobuki()
        self.odom_helper = OdomHelper(self._move_robot_internal)
        
        # Movement parameters
        self.following_speed = 0.15
        self.turning_speed = 0.15
        self.angular_gain = 0.001
        self.max_angular_speed = 0.8
        
        # Avoidance parameters
        self.avoiding_counter = 0
        self.avoiding_threshold = 5
        self.avoidance_forward_duration = 2.0
        self.avoidance_forward_speed = 0.1
        self.avoidance_turning_speed = 0.5
        
        # Detection parameters
        self.threshold_center_wp = 0.2
        
        # State management
        self.current_state = self.STATES['SEARCHING']
        self.previous_state = None

    def update_state(self, person_detected, person_center, obstacle_front):
        """Update robot state based on sensor inputs"""
        if obstacle_front:
            if person_detected and self._is_person_centered(person_center):
                self.current_state = self.STATES['WAITING']
            else:
                self.current_state = self.STATES['AVOIDING']
        else:
            self.current_state = self.STATES['FOLLOWING'] if person_detected else self.STATES['SEARCHING']

    def generate_twist(self, person_detected, person_center, image_width, obstacle_front, obstacle_left):
        """Generate movement command based on current situation"""
        self.update_state(person_detected, person_center, obstacle_front)
        
        # State machine for movement generation
        twist_generators = {
            self.STATES['WAITING']: self._generate_waiting_twist,
            self.STATES['AVOIDING']: lambda: self._generate_avoiding_twist(obstacle_left, person_center),
            self.STATES['FOLLOWING']: lambda: self._generate_following_twist(person_center, image_width),
            self.STATES['SEARCHING']: self._generate_searching_twist
        }
        
        twist = twist_generators[self.current_state]()
        self.previous_state = self.current_state
        
        return twist

    def _generate_waiting_twist(self):
        """Generate twist for waiting state"""
        twist = Twist()
        rospy.loginfo("WAITING")
        return twist

    def _generate_avoiding_twist(self, obstacle_left, person_center):
        """Generate twist for obstacle avoidance"""
        twist = Twist()
        
        # Increment counter if staying in avoiding state
        if self.previous_state == self.STATES['AVOIDING']:
            self.avoiding_counter += 1
        else:
            self.avoiding_counter = 1
        
        # Execute avoidance maneuver when threshold is reached
        if self.avoiding_counter >= self.avoiding_threshold:
            rospy.loginfo("AVOIDING obstacle")
            self._execute_obstacle_avoidance(obstacle_left, person_center)
            self.avoiding_counter = 0
        
        return twist

    def _generate_following_twist(self, person_center, image_width):
        """Generate twist for person following"""
        twist = Twist()
        
        if person_center is not None:
            # Calculate angular velocity based on person position
            error_x = person_center[0] - image_width // 2
            angular_vel = self._clamp(-self.angular_gain * error_x, 
                                    -self.max_angular_speed, 
                                    self.max_angular_speed)
            
            twist.linear.x = self.following_speed
            twist.angular.z = angular_vel
            rospy.loginfo(f"FOLLOWING person - Lin: {twist.linear.x:.2f}, Ang: {twist.angular.z:.2f}")
        
        return twist

    def _generate_searching_twist(self):
        """Generate twist for searching state"""
        twist = Twist()
        twist.angular.z = self.turning_speed
        rospy.loginfo("SEARCHING for person")
        return twist

    def _is_person_centered(self, person_center):
        """Check if person is centered in the image"""
        if person_center is None:
            return False
        
        error_x = abs(self.CENTER_WAITING_PERSON[0] - person_center[0])
        normalized_error = error_x / abs(self.CENTER_WAITING_PERSON[0])
        
        return normalized_error <= self.threshold_center_wp

    def _execute_obstacle_avoidance(self, obstacle_left, person_center):
        """
        Simplified obstacle avoidance: rotate away from obstacle, 
        move forward, then rotate back
        """
        # Determine rotation direction
        rotation_direction = self._get_avoidance_direction(obstacle_left, person_center)
        rotation_angle = math.radians(90 * rotation_direction)
        
        # Execute avoidance maneuver
        self.odom_helper.rotate_by_angle(rotation_angle, self.avoidance_turning_speed)
        self._move_forward_for_duration(self.avoidance_forward_speed, self.avoidance_forward_duration)
        self.odom_helper.rotate_by_angle(-rotation_angle, self.avoidance_turning_speed)
        self.stop_robot()

    def _get_avoidance_direction(self, obstacle_left, person_center):
        """
        Determine which direction to rotate for obstacle avoidance
        Returns: 1 for left rotation, -1 for right rotation
        """
        # Default: rotate left
        direction = 1
        
        # If person is detected, consider their position
        if person_center is not None:
            person_is_left = person_center[0] < self.CENTER_WAITING_PERSON[0]
            direction = 1 if person_is_left else -1
        
        # If obstacle is on the left, rotate right instead
        if obstacle_left:
            direction = -1
            
        return direction

    def _move_forward_for_duration(self, speed, duration):
        """Move robot forward at given speed for specified duration"""
        twist = Twist()
        twist.linear.x = speed
        
        end_time = rospy.Time.now() + rospy.Duration(duration)
        rate = rospy.Rate(10)
        
        while rospy.Time.now() < end_time and not rospy.is_shutdown():
            self._move_robot_internal(twist)
            rate.sleep()

    def _move_robot_internal(self, twist):
        """Internal method for robot movement (used by odom_helper)"""
        self.move_kobuki.move_robot(twist)

    def _clamp(self, value, min_val, max_val):
        """Clamp value between min and max"""
        return max(min_val, min(value, max_val))

    # Public interface methods
    def move_robot(self, twist):
        """Send movement command to robot"""
        self.move_kobuki.move_robot(twist)

    def stop_robot(self):
        """Stop the robot"""
        self.move_robot(Twist())

    def cleanup(self):
        """Clean up resources"""
        self.stop_robot()
        self.move_kobuki.clean_class()