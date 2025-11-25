#!/usr/bin/env python3
import rospy
import tf2_ros
import geometry_msgs.msg
import visualization_msgs.msg
import math
import threading  
from geometry_msgs.msg import Pose2D

# -------------------------------------------------
# Helper: distance between two points
# -------------------------------------------------
def dist(a, b):
    return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)

# -------------------------------------------------
# Shared goal state (updated by delivery.cpp)
# -------------------------------------------------
goal_lock = threading.Lock()
goal_xy = [3.0, 2.0]  # default goal until /delivery_goal gives us one

subgoal_lock = threading.Lock()
subgoal_xy = [3.0, 2.0]
have_subgoal = False


def goal_callback(msg):
    """Update goal_xy whenever /delivery_goal publishes a new Point."""
    with goal_lock:
        goal_xy[0] = msg.x
        goal_xy[1] = msg.y

def next_goal_callback(msg):
    global have_subgoal
    with subgoal_lock:
        subgoal_xy[0] = msg.x
        subgoal_xy[1] = msg.y
        have_subgoal = True


if __name__ == "__main__":
    rospy.init_node("nav_sim")

    br = tf2_ros.TransformBroadcaster()

    robot_marker_pub = rospy.Publisher("robot_marker", visualization_msgs.msg.Marker, queue_size=10)
    goal_marker_pub  = rospy.Publisher("goal_marker",  visualization_msgs.msg.Marker, queue_size=10)
    obs_marker_pub   = rospy.Publisher("obstacle_markers", visualization_msgs.msg.Marker, queue_size=10)
    trail_marker_pub = rospy.Publisher("trail_marker", visualization_msgs.msg.Marker, queue_size=10)

    # NEW: listen for live delivery goals from delivery.cpp
    goal_sub = rospy.Subscriber("delivery_goal",
                                geometry_msgs.msg.Point,
                                goal_callback)
    
    next_goal_sub = rospy.Subscriber("next_goal",
                                 geometry_msgs.msg.Point,
                                 next_goal_callback)


    rate = rospy.Rate(15)  # 15 Hz looks smooth

    # -------------------------
    # World setup
    # -------------------------
    # Robot start pose
    x = 0.0
    y = 0.0
    theta = 0.0  # heading (radians)

    # Obstacles: list of (x,y,radius)
    obstacles = [
        (1.2, 0.5, 0.4),
        (2.0, 1.5, 0.4),
        (2.5, 0.5, 0.4),
    ]

    # Breadcrumb trail of visited positions
    trail_points = []

    # Movement tuning
    base_speed     = 0.03   # forward step size per tick
    avoid_strength = 0.6    # how hard we steer away from obstacles
    avoid_range    = 0.8    # start avoiding when within this distance
    
    pose_pub = rospy.Publisher("/robot_pose", Pose2D, queue_size=10)

    while not rospy.is_shutdown():
        # -------------------------------------------------
        # 0. Read the current goal safely
        # -------------------------------------------------
        with subgoal_lock:
            if have_subgoal:
                gx = subgoal_xy[0]
                gy = subgoal_xy[1]
            else:
                with goal_lock:
                    gx = goal_xy[0]
                    gy = goal_xy[1]


        # Distance to goal
        to_goal_x_full = gx - x
        to_goal_y_full = gy - y
        goal_dist_full = math.sqrt(to_goal_x_full**2 + to_goal_y_full**2)


        # -------------------------------------------------
        # 1. Attraction toward goal ("go this way")
        # -------------------------------------------------
        to_goal_x = gx - x
        to_goal_y = gy - y
        goal_dist = math.sqrt(to_goal_x**2 + to_goal_y**2)

        if goal_dist > 0.0001:
            to_goal_x /= goal_dist
            to_goal_y /= goal_dist

        # -------------------------------------------------
        # 2. Repulsion from obstacles ("not through that blob")
        # -------------------------------------------------
        repel_x = 0.0
        repel_y = 0.0
        for (ox, oy, r) in obstacles:
            d = dist((x, y), (ox, oy))
            if d < avoid_range and d > 0.0001:
                push = (avoid_range - d) / avoid_range  # 0..1
                dir_x = x - ox
                dir_y = y - oy
                mag = math.sqrt(dir_x**2 + dir_y**2)
                if mag > 0.0001:
                    dir_x /= mag
                    dir_y /= mag
                    repel_x += dir_x * push
                    repel_y += dir_y * push

        # -------------------------------------------------
        # 3. Combine attraction + repulsion
        # -------------------------------------------------
        move_x = to_goal_x + avoid_strength * repel_x
        move_y = to_goal_y + avoid_strength * repel_y

        mv_mag = math.sqrt(move_x**2 + move_y**2)
        if mv_mag > 0.0001:
            move_x /= mv_mag
            move_y /= mv_mag

        # -------------------------------------------------
        # 4. Advance robot
        # -------------------------------------------------
        x += move_x * base_speed
        y += move_y * base_speed

        theta = math.atan2(move_y, move_x)

        # Publish robot pose
        pose = Pose2D()
        pose.x = x
        pose.y = y
        pose.theta = theta
        pose_pub.publish(pose)
        # -------------------------------------------------
        # 5. Leave breadcrumb trail
        # -------------------------------------------------
        trail_points.append((x, y))
        if len(trail_points) > 200:
            trail_points.pop(0)

        # -------------------------------------------------
        # 6. Broadcast TF: map -> base_link
        # -------------------------------------------------
        tf_msg = geometry_msgs.msg.TransformStamped()
        tf_msg.header.stamp = rospy.Time.now()
        tf_msg.header.frame_id = "map"
        tf_msg.child_frame_id = "base_link"
        tf_msg.transform.translation.x = x
        tf_msg.transform.translation.y = y
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation.x = 0.0
        tf_msg.transform.rotation.y = 0.0
        tf_msg.transform.rotation.z = math.sin(theta/2.0)
        tf_msg.transform.rotation.w = math.cos(theta/2.0)
        br.sendTransform(tf_msg)

        # -------------------------------------------------
        # 7. Robot marker (yellow arrow) - in map frame at (x, y)
        # -------------------------------------------------
        robot_marker = visualization_msgs.msg.Marker()
        robot_marker.header.frame_id = "map"   # changed from "base_link"
        robot_marker.header.stamp = rospy.Time.now()
        robot_marker.ns = "robot"
        robot_marker.id = 0
        robot_marker.type = visualization_msgs.msg.Marker.ARROW
        robot_marker.action = visualization_msgs.msg.Marker.ADD
        robot_marker.scale.x = 0.5   # length
        robot_marker.scale.y = 0.15  # width
        robot_marker.scale.z = 0.05  # thickness
        robot_marker.color.r = 1.0
        robot_marker.color.g = 1.0
        robot_marker.color.b = 0.0
        robot_marker.color.a = 1.0

        # orientation from theta
        robot_marker.pose.orientation.z = math.sin(theta/2.0)
        robot_marker.pose.orientation.w = math.cos(theta/2.0)

        # put robot at its simulated position in map frame
        robot_marker.pose.position.x = x       # changed from 0.0
        robot_marker.pose.position.y = y       # changed from 0.0
        robot_marker.pose.position.z = 0.0
        robot_marker_pub.publish(robot_marker)


        # -------------------------------------------------
        # 8. Goal marker (green cube where gx, gy is)
        # -------------------------------------------------
        goal_marker = visualization_msgs.msg.Marker()
        goal_marker.header.frame_id = "map"
        goal_marker.header.stamp = rospy.Time.now()
        goal_marker.ns = "goal"
        goal_marker.id = 1
        goal_marker.type = visualization_msgs.msg.Marker.CUBE
        goal_marker.action = visualization_msgs.msg.Marker.ADD
        goal_marker.scale.x = 0.3
        goal_marker.scale.y = 0.3
        goal_marker.scale.z = 0.1
        goal_marker.color.r = 0.0
        goal_marker.color.g = 1.0
        goal_marker.color.b = 0.0
        goal_marker.color.a = 1.0
        goal_marker.pose.position.x = gx
        goal_marker.pose.position.y = gy
        goal_marker.pose.position.z = 0.0
        goal_marker.pose.orientation.w = 1.0
        goal_marker_pub.publish(goal_marker)

        # -------------------------------------------------
        # 9. Obstacles (red blobs)
        # -------------------------------------------------
        obs_marker = visualization_msgs.msg.Marker()
        obs_marker.header.frame_id = "map"
        obs_marker.header.stamp = rospy.Time.now()
        obs_marker.ns = "obstacles"
        obs_marker.id = 2
        obs_marker.type = visualization_msgs.msg.Marker.SPHERE_LIST
        obs_marker.action = visualization_msgs.msg.Marker.ADD
        obs_marker.scale.x = 0.8
        obs_marker.scale.y = 0.8
        obs_marker.scale.z = 0.2
        obs_marker.color.r = 1.0
        obs_marker.color.g = 0.0
        obs_marker.color.b = 0.0
        obs_marker.color.a = 0.6
        obs_marker.points = []
        for (ox, oy, r) in obstacles:
            p = geometry_msgs.msg.Point()
            p.x = ox
            p.y = oy
            p.z = 0.0
            obs_marker.points.append(p)
        obs_marker_pub.publish(obs_marker)

        # -------------------------------------------------
        # 10. Trail (blue path)
        # -------------------------------------------------
        trail = visualization_msgs.msg.Marker()
        trail.header.frame_id = "map"
        trail.header.stamp = rospy.Time.now()
        trail.ns = "trail"
        trail.id = 3
        trail.type = visualization_msgs.msg.Marker.LINE_STRIP
        trail.action = visualization_msgs.msg.Marker.ADD
        trail.scale.x = 0.05
        trail.color.r = 0.0
        trail.color.g = 0.5
        trail.color.b = 1.0
        trail.color.a = 1.0
        trail.points = []
        for (tx, ty) in trail_points:
            pt = geometry_msgs.msg.Point()
            pt.x = tx
            pt.y = ty
            pt.z = 0.01
            trail.points.append(pt)
        trail_marker_pub.publish(trail)

        rate.sleep()
