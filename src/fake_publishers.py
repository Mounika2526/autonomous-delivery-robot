#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

def main():
    rospy.init_node("fake_publishers")

    sender_pub = rospy.Publisher("senderLocation", String, queue_size=10)
    receiver_pub = rospy.Publisher("receiverLocation", String, queue_size=10)

    rate = rospy.Rate(0.5)   # publish every 2 seconds

    sender_msg = String(data="Location A")
    receiver_msg = String(data="Location B")

    while not rospy.is_shutdown():
        sender_pub.publish(sender_msg)
        receiver_pub.publish(receiver_msg)
        rate.sleep()

if __name__ == "__main__":
    main()
