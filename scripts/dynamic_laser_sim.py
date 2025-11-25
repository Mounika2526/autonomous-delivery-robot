#!/usr/bin/env python3
import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose2D

class DynamicLaserSim:
    def __init__(self):
        self.scan_pub = rospy.Publisher("/scan", LaserScan, queue_size=10)
        self.pose_sub = rospy.Subscriber("/robot_pose", Pose2D, self.pose_cb)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.obstacles = [
            {"x": 0.5, "y": 0.0, "vx": 0.00, "vy": 0.00},  # very close
            {"x": 1.0, "y": 0.2, "vx": 0.00, "vy": 0.00},  # near front-right
            {"x": 1.0, "y": -0.2, "vx": 0.00, "vy": 0.00}  # near front-left
            ]

        self.rate = rospy.Rate(10)

    def pose_cb(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.theta = msg.theta

    def update_obstacles(self):
        for obs in self.obstacles:
            obs["x"] += obs["vx"]
            obs["y"] += obs["vy"]
            if obs["x"] > 5 or obs["x"] < -5:
                obs["vx"] *= -1

    def compute_scan(self):
        scan = LaserScan()
        scan.header.frame_id = "laser"
        scan.angle_min = -math.pi/2
        scan.angle_max = math.pi/2
        scan.angle_increment = (scan.angle_max - scan.angle_min) / 480.0
        scan.range_min = 0.05
        scan.range_max = 6.0

        ranges = []
        for i in range(480):
            angle = scan.angle_min + i * scan.angle_increment + self.theta
            dx = math.cos(angle)
            dy = math.sin(angle)
            best = scan.range_max

            for obs in self.obstacles:
                vx = obs["x"] - self.x
                vy = obs["y"] - self.y
                t = vx * dx + vy * dy
                if t < 0:
                    continue

                px = self.x + t * dx
                py = self.y + t * dy
                dist_sq = (px - obs["x"])**2 + (py - obs["y"])**2
                if dist_sq <= (0.3**2):
                    if t < best:
                        best = t

            ranges.append(best)

        scan.ranges = ranges
        return scan

    def run(self):
        while not rospy.is_shutdown():
            self.update_obstacles()
            scan = self.compute_scan()
            self.scan_pub.publish(scan)
            self.rate.sleep()

if __name__ == "__main__":
    rospy.init_node("dynamic_laser_sim")
    node = DynamicLaserSim()
    node.run()
