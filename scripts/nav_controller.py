#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Int8   # <<< NEW

class NavController:
    def __init__(self):
        # Current robot pose
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # Current goal from planner
        self.goal = None          # geometry_msgs/Point
        self.scan = None          # sensor_msgs/LaserScan

        # Track which setpoint we are following
        self.current_setpoint = -1

        # Publishers / subscribers
        self.cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("odom", Odometry, self.odom_cb)
        # We no longer depend on /next_goal for sim, but you can keep this if you want
        # rospy.Subscriber("/next_goal", Point, self.goal_cb)
        rospy.Subscriber("/scan", LaserScan, self.scan_cb)
        rospy.Subscriber("setpoint", Int8, self.setpoint_cb)   # <<< NEW

        # Parameters
        self.max_lin = rospy.get_param("~max_linear_speed", 0.4)
        self.max_ang = rospy.get_param("~max_angular_speed", 1.0)
        self.goal_tolerance = rospy.get_param("~goal_tolerance", 0.15)
        self.obstacle_dist = rospy.get_param("~obstacle_distance", 0.8)

    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.yaw = yaw

    def goal_cb(self, msg):
        # Not used in sim anymore, but kept for compatibility
        self.goal = msg

    def scan_cb(self, msg):
        self.scan = msg

        # ---------- map /setpoint -> a goal in Gazebo ----------
    def setpoint_cb(self, msg: Int8):
        sp = msg.data
        if sp == self.current_setpoint:
            return

        self.current_setpoint = sp
        rospy.loginfo("NavController: new setpoint %d", sp)

        # Map setpoints to fixed positions in the odom frame.
        # Coordinates must match delivery.cpp -> get_location_xy()
        if sp == 0:         # Location A (placeholder for now)
            self.goal = Point(0.0, 0.0, 0.0)

        elif sp == 1:       # Location B (placeholder for now)
            self.goal = Point(0.0, 0.0, 0.0)

        elif sp == 2:       # Location C
            # MATCH: get_location_xy("Location C") -> (0.5, -0.8)
            self.goal = Point(0.5, -0.8, 0.0)

        elif sp == 3:       # Location D
            # MATCH: get_location_xy("Location D") -> (3.0, 0.8)
            self.goal = Point(3.0, 0.8, 0.0)

        else:
            # -1 or anything else: no active goal -> stop
            self.goal = None
            rospy.loginfo("NavController: no active goal for setpoint %d", sp)

    def compute_obstacle_bias(self):
        if self.scan is None:
            return 0.0

        ranges = list(self.scan.ranges)
        n = len(ranges)
        if n < 3:
            return 0.0

        third = n // 3

        right = [r for r in ranges[:third] if 0.05 < r < self.obstacle_dist]
        front = [r for r in ranges[third:2*third] if 0.05 < r < self.obstacle_dist]
        left  = [r for r in ranges[2*third:] if 0.05 < r < self.obstacle_dist]

        # NEW: if nothing is close → no avoidance needed
        if not right and not front and not left:
            return 0.0

        # If obstacle directly in front
        if front:
            avg_left = min(left) if left else 99
            avg_right = min(right) if right else 99

            if avg_left > avg_right:
                rospy.loginfo("Avoiding obstacle → turn LEFT")
                return +1.5       # strong left
            else:
                rospy.loginfo("Avoiding obstacle → turn RIGHT")
                return -1.5       # strong right

        # If obstacle only on left → turn right
        if left and not front:
            return -1.0

        # If obstacle only on right → turn left
        if right and not front:
            return +1.0

        return 0.0
    

    def spin(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            twist = Twist()

            if self.goal is None:
                # No goal yet
                self.cmd_pub.publish(twist)
                rate.sleep()
                continue

            # Vector to goal
            dx = self.goal.x - self.x
            dy = self.goal.y - self.y
            dist = math.hypot(dx, dy)

            # Close enough → stop
            if dist < self.goal_tolerance:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.cmd_pub.publish(twist)
                rospy.loginfo("NavController: goal reached at (%.2f, %.2f)", self.x, self.y)
                self.goal = None
                rate.sleep()
                continue

            close_to_goal = dist < 0.4

            # Desired heading
            desired_yaw = math.atan2(dy, dx)
            yaw_error = desired_yaw - self.yaw
            # Normalize to [-pi, pi]
            yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))

            # Small deadband so robot drives straight when nearly aligned
            if abs(yaw_error) < 0.05:
                yaw_error = 0.0

            # Base controller: face goal and move
            lin = self.max_lin * max(0.0, 1.0 - abs(yaw_error))
            ang = self.max_ang * yaw_error

            # Obstacle avoidance correction
            if close_to_goal:
                bias = 0.0
            else:
                bias = self.compute_obstacle_bias()

            if bias != 0.0:
                lin *= 0.2          # slow down more when avoiding
                ang = bias * 1.5    # strong turn


            # Clamp angular speed
            if ang > self.max_ang:
                ang = self.max_ang
            elif ang < -self.max_ang:
                ang = -self.max_ang

            twist.linear.x = lin
            twist.angular.z = ang

            self.cmd_pub.publish(twist)
            rate.sleep()

if __name__ == "__main__":
    rospy.init_node("nav_controller")
    node = NavController()
    node.spin()
