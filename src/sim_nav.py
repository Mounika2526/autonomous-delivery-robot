#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int8
from geometry_msgs.msg import Twist

class SimNav:
    def __init__(self):
        self.setpoint = -1

        # ✅ publish to cmd_vel_nav (navigation output)
        self.cmd_pub = rospy.Publisher("cmd_vel_nav", Twist, queue_size=10)

        # listen to the integer setpoint from delivery_adr
        rospy.Subscriber("setpoint", Int8, self.setpoint_cb)

    def setpoint_cb(self, msg: Int8):
        self.setpoint = msg.data
        rospy.loginfo("SimNav: received setpoint %d", self.setpoint)

    def spin(self):
        rate = rospy.Rate(10)   # 10 Hz
        while not rospy.is_shutdown():
            twist = Twist()

            # Map setpoints to simple forward/backward motion.
            # 0,2  -> forward   (e.g., A, C)
            # 1,3  -> backward  (e.g., B, D)
            if self.setpoint in (0, 2):
                twist.linear.x = 0.2
                twist.angular.z = 0.0
            elif self.setpoint in (1, 3):
                twist.linear.x = -0.2
                twist.angular.z = 0.0
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.0

            self.cmd_pub.publish(twist)
            rate.sleep()

if __name__ == "__main__":
    rospy.init_node("sim_nav")
    SimNav().spin()
