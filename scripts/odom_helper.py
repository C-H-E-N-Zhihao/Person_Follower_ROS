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

    def rotate_relative(self, angle_rad, angular_speed=0.5):
        """
        Rotates the robot exactly 'angle_rad' radians from its current orientation.
        Always take the shortest path and avoid unnecessary turns.
        """
        
        start_yaw = self.get_yaw()
        direction = math.copysign(1.0, angle_rad)
        remaining = abs(angle_rad)

        twist = Twist()
        twist.angular.z = direction * abs(angular_speed)

        rate = rospy.Rate(10)  # Hz
        prev_yaw = start_yaw
        rotated = 0.0

        while not rospy.is_shutdown() and rotated < remaining:
            current_yaw = self.get_yaw()

            # Δyaw desde la iteración anterior (con wrap-around)
            delta_yaw = math.atan2(math.sin(current_yaw - prev_yaw), math.cos(current_yaw - prev_yaw))
            rotated += abs(delta_yaw)
            prev_yaw = current_yaw

            self.move_fn(twist)
            rate.sleep()

        self.move_fn(Twist())  # stop

