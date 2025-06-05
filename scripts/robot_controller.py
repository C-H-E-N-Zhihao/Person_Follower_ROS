#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import math
from odom_helper import OdomHelper
from move_robot import MoveKobuki

CENTER_WAITING_PERSON = (320, 177)

class RobotController:
    def __init__(self):
        self.move_kobuki = MoveKobuki()

        # Initialize OdomHelper for odometry-based movements
        self.odom_helper = OdomHelper(self.move_robot)
        
        # Control parameters
        self.following_speed = 0.15
        self.turning_speed = 0.15
        self.angular_gain = 0.001
        self.max_angular_speed = 0.8

        # Avoidance parameters
        self.avoiding_counter = 0
        self.avoiding_threshold = 5  # number of cycles before activating evasion
        self.previous_state = None

        self.center_waiting_person = (320, 177)
        self.threshold_center_wp = 0.2
        
        # State
        self.state = "SEARCHING"  # SEARCHING, FOLLOWING, AVOIDING
    
    def update_state(self, person_detected, person_center, obstacle_front):
        """Update robot state based on sensors"""
        
        if obstacle_front:
            person_at_center = self.person_at_center_x(self.center_waiting_person, person_center, self.threshold_center_wp)
            
            if person_detected and person_at_center:
                self.state = "WAITING"
            
            elif not(person_detected) or not(person_at_center):
                self.state = "AVOIDING"
        else:
            if person_detected:
                self.state = "FOLLOWING"  
                
            else:
                self.state = "SEARCHING"
    
    def generate_twist(self, person_detected, person_center, image_width, obstacle_front, obstacle_left):
        """Generate movement command based on current situation"""
        twist = Twist()
        
        # Update state
        self.update_state(person_detected, person_center, obstacle_front)
        
        if self.state == "WAITING": 
            print(person_center)
            twist.angular.z = 0.0
            twist.linear.x = 0.0  
            rospy.loginfo("WAITING")
            
        
        elif self.state == "AVOIDING":
            # Turn right and back up slightly
            #twist.angular.z = self.turning_speed  # Turn right
            #twist.linear.x = 0.0  # Stop moving forward

            # Incrementar contador si sigue en el mismo estado
            if self.previous_state == "AVOIDING":
                self.avoiding_counter += 1
            else:
                self.avoiding_counter = 1  # reset if just entered AVOIDING
            
            # Ejecutar evasión solo si el contador alcanza el umbral
            if self.avoiding_counter >= self.avoiding_threshold:
                rospy.loginfo("AVOIDING obstacle")
                self.obstacle_avoidance_loop(obstacle_left=obstacle_left)
                self.avoiding_counter = 0  # reset after avoidance
        
        elif self.state == "FOLLOWING" and person_center is not None:
            # Follow person
            person_center_x = person_center[0]
            error_x = person_center_x - image_width // 2
            angular_vel = -self.angular_gain * error_x
            
            # Limit angular velocity
            if angular_vel > self.max_angular_speed:
                angular_vel = self.max_angular_speed
            elif angular_vel < -self.max_angular_speed:
                angular_vel = -self.max_angular_speed
            
            # Move forward at constant speed
            twist.linear.x = self.following_speed
            twist.angular.z = angular_vel
            rospy.loginfo(f"FOLLOWING person - Lin: {twist.linear.x:.2f}, Ang: {twist.angular.z:.2f}")
        
        elif self.state == "SEARCHING":  # SEARCHING
            # Rotate to search for person
            twist.angular.z = self.turning_speed
            twist.linear.x = 0.0
            rospy.loginfo("SEARCHING for person")
        
        self.previous_state = self.state  # Update previous state for next iteration

        return twist

    def person_at_center_x(self, ground_truth, observed, threshold=0.1):
        """
        Checks if the observed person is at the center based on x-axis error.
        """
        
        if observed == None:
            return False

        # Calculate absolute error in x
        error_x = abs(ground_truth[0] - observed[0])

        # Normalize by ground truth x to make it scale-independent
        norm_factor = abs(ground_truth[0])

        normalized_error = error_x / norm_factor

        return normalized_error <= threshold
        
    def obstacle_avoidance_loop(self, obstacle_left=False):
        """
        Reactive obstacle avoidance using odometry-based rotation.
        1. Rotate 90° to the left
        2. Move forward
        3. Rotate 90° to the right
        """
        C = -1 if obstacle_left else 1

        # Step 1: Rotate 90° left (positive angle)
        self.odom_helper.rotate_by_angle(math.radians(90 * C))

        # Step 2: Move forward
        twist_forward = Twist()
        twist_forward.linear.x = 0.1
        duration = rospy.Duration(2.0)
        rate = rospy.Rate(10)
        start_time = rospy.Time.now()
        while rospy.Time.now() - start_time < duration and not rospy.is_shutdown():
            self.move_robot(twist_forward)
            rate.sleep()

        # Step 3: Rotate 90° right (negative angle)
        self.odom_helper.rotate_by_angle(math.radians(-90 * C))

        self.stop_robot()


    	
    def move_robot(self, twist):
        """Send movement command to robot"""
        self.move_kobuki.move_robot(twist)
    
    def stop_robot(self):
        """Stop the robot"""
        twist = Twist()  # All zeros
        self.move_kobuki.move_robot(twist)
    
    def cleanup(self):
        """Clean up resources"""
        self.stop_robot()
        self.move_kobuki.clean_class()
        
