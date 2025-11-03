#!/usr/bin/env python3
import rospy
if __name__ == "__main__":
    rospy.init_node("fake_laser")
    rospy.loginfo("Dummy fake_laser node running...")
    rospy.spin()
