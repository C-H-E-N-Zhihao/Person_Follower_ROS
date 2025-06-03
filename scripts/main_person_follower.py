#!/usr/bin/env python3

import rospy
from obstacle_detector import ObstacleDetector
from person_detector import PersonDetector
from robot_controller import RobotController


class PersonFollowerMain:
    def __init__(self):
        # Initialize components
        self.obstacle_detector = ObstacleDetector()
        self.person_detector = PersonDetector()
        self.robot_controller = RobotController()
        
        rospy.loginfo("Person Follower started")
    
    def run(self):
        """Main control loop"""
        rate = rospy.Rate(20)  # 10 Hz
        
        while not rospy.is_shutdown():
            # Get sensor data
            obstacle_front = self.obstacle_detector.is_obstacle_front()
            person_detected, person_center_x, image_width = self.person_detector.get_person_info()
            
            # Generate and send movement command
            twist = self.robot_controller.generate_twist(
                person_detected, person_center_x, image_width, obstacle_front
            )
            self.robot_controller.move_robot(twist)
            
            rate.sleep()
    
    def cleanup(self):
        """Clean up all components"""
        self.person_detector.cleanup()
        self.robot_controller.cleanup()
        rospy.loginfo("Person Follower stopped")


def main():
    rospy.init_node('person_follower_node', anonymous=True)
    
    try:
        follower = PersonFollowerMain()
        
        def shutdown_hook():
            follower.cleanup()
        
        rospy.on_shutdown(shutdown_hook)
        
        # Run main loop
        follower.run()
        
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Error: {e}")


if __name__ == '__main__':
    main()
