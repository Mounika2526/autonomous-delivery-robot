#!/usr/bin/env python3
import rospy
import tf2_ros
import geometry_msgs.msg
import math
import time

if __name__ == "__main__":
    rospy.init_node("pose_broadcaster")

    br = tf2_ros.TransformBroadcaster()

    rate = rospy.Rate(10)  # 10 Hz

    x = 0.0
    y = 0.0
    theta = 0.0

    start_time = time.time()

    while not rospy.is_shutdown():
        # Make the robot move in a small circle over time just for visualization
        t = time.time() - start_time
        x = 0.5 * math.cos(t * 0.2)
        y = 0.5 * math.sin(t * 0.2)
        theta = t * 0.2  # yaw in radians

        tf_msg = geometry_msgs.msg.TransformStamped()
        tf_msg.header.stamp = rospy.Time.now()
        tf_msg.header.frame_id = "map"
        tf_msg.child_frame_id = "base_link"

        tf_msg.transform.translation.x = x
        tf_msg.transform.translation.y = y
        tf_msg.transform.translation.z = 0.0

        # Convert yaw to quaternion
        qz = math.sin(theta / 2.0)
        qw = math.cos(theta / 2.0)

        tf_msg.transform.rotation.x = 0.0
        tf_msg.transform.rotation.y = 0.0
        tf_msg.transform.rotation.z = qz
        tf_msg.transform.rotation.w = qw

        br.sendTransform(tf_msg)

        rate.sleep()
