import rospy
import math
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class OdomHelper:
    def __init__(self, move_fn):
        self.latest_odom = None
        self.move_fn = move_fn  # función externa para mover el robot
        rospy.Subscriber("/odom", Odometry, self.odom_callback)

    def odom_callback(self, msg):
        self.latest_odom = msg

    def get_yaw(self):
        if not self.latest_odom:
            rospy.logwarn("No odometry received.")
            return 0.0
        q = self.latest_odom.pose.pose.orientation
        return tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

    def rotate_by_angle(self, angle_rad, angular_speed=0.3):
        start_yaw = self.get_yaw()
        target_yaw = start_yaw + angle_rad

        # Normalize target yaw to [-pi, pi]
        target_yaw = math.atan2(math.sin(target_yaw), math.cos(target_yaw))

        twist = Twist()
        twist.angular.z = math.copysign(angular_speed, angle_rad)

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            current_yaw = self.get_yaw()
            delta = math.atan2(math.sin(target_yaw - current_yaw), math.cos(target_yaw - current_yaw))
            if abs(delta) < 0.02:
                break
            self.move_fn(twist)
            rate.sleep()

        # Stop the robot after rotation
        self.move_fn(Twist())
