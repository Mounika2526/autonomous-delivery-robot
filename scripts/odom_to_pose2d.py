#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D
from tf.transformations import euler_from_quaternion

class OdomToPose2D:
    def __init__(self):
        self.pub = rospy.Publisher("/robot_pose", Pose2D, queue_size=10)
        rospy.Subscriber("odom", Odometry, self.odom_cb)

    def odom_cb(self, msg):
        pose2d = Pose2D()
        pose2d.x = msg.pose.pose.position.x
        pose2d.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        pose2d.theta = yaw

        self.pub.publish(pose2d)

if __name__ == "__main__":
    rospy.init_node("odom_to_pose2d")
    OdomToPose2D()
    rospy.spin()