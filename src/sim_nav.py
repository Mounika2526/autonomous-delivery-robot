#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int8
from geometry_msgs.msg import Twist

class SimNav:
    def __init__(self):
        self.setpoint = -1
        self.cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("setpoint", Int8, self.setpoint_cb)

    def setpoint_cb(self, msg):
        self.setpoint = msg.data

    def spin(self):
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            twist = Twist()

            if self.setpoint == 0:      # Location A
                twist.linear.x = 1.0
                twist.angular.z = 0.0
            elif self.setpoint == 1:    # Location B
                twist.linear.x = -1.0
                twist.angular.z = 0.0
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.0

            self.cmd_pub.publish(twist)
            rate.sleep()

if __name__ == "__main__":
    rospy.init_node("sim_nav")
    SimNav().spin()
