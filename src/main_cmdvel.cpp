#include <cmath>
#include <iostream>
#include <vector>

#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Point.h"
#include "nav_msgs/Odometry.h"

// Tracks current delivery goal (x,y) from delivery.cpp
class GoalTracker
{
public:
    ros::Subscriber sub;
    double x{0.0};
    double y{0.0};
    bool has_goal{false};

    GoalTracker(ros::NodeHandle* nh)
    {
        int q = 10;
        sub = nh->subscribe("delivery_goal", q,
                            &GoalTracker::goalCallback, this);
    }

    void goalCallback(const geometry_msgs::Point::ConstPtr& msg)
    {
        x = msg->x;
        y = msg->y;
        has_goal = true;
    }
};

// Tracks robot pose (x,y,yaw) from /odom
class PoseTracker
{
public:
    ros::Subscriber sub;
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
    bool has_pose{false};

    PoseTracker(ros::NodeHandle* nh)
    {
        int q = 50;
        sub = nh->subscribe("odom", q,
                            &PoseTracker::odomCallback, this);
    }

    static double yawFromQuaternion(const geometry_msgs::Quaternion& q)
    {
        double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        return std::atan2(siny_cosp, cosy_cosp);
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        x = msg->pose.pose.position.x;
        y = msg->pose.pose.position.y;
        yaw = yawFromQuaternion(msg->pose.pose.orientation);
        has_pose = true;
    }
};

// Turns /delivery_goal + /odom + /setpoint into /cmd_vel_nav
class NavController
{
public:
    ros::Publisher cmd_pub;
    ros::Subscriber setpoint_sub;

    int setpoint{-1};  // -1 = idle / no navigation

    GoalTracker* goal;
    PoseTracker* pose;

    NavController(ros::NodeHandle* nh, GoalTracker* g, PoseTracker* p)
        : goal(g), pose(p)
    {
        int q = 10;
        cmd_pub = nh->advertise<geometry_msgs::Twist>("cmd_vel_nav", q);
        setpoint_sub = nh->subscribe("setpoint", q,
                                     &NavController::setpointCallback, this);
    }

    void setpointCallback(const std_msgs::Int8::ConstPtr& msg)
    {
        setpoint = msg->data;
    }

    static double normalizeAngle(double a)
    {
        while (a > M_PI)  a -= 2.0 * M_PI;
        while (a < -M_PI) a += 2.0 * M_PI;
        return a;
    }

    void step()
    {
        geometry_msgs::Twist cmd;

        // If we don't have everything we need, stop
        if (setpoint < 0 || !goal->has_goal || !pose->has_pose)
        {
            cmd.linear.x  = 0.0;
            cmd.angular.z = 0.0;
            cmd_pub.publish(cmd);
            ROS_WARN_THROTTLE(1.0,
                "NavGoal: idle (setpoint=%d, has_goal=%d, has_pose=%d)",
                setpoint, (int)goal->has_goal, (int)pose->has_pose);
            return;
        }

        // Vector from robot to goal
        double dx = goal->x - pose->x;
        double dy = goal->y - pose->y;
        double dist = std::sqrt(dx*dx + dy*dy);

        // Desired heading to goal
        double desired_yaw   = std::atan2(dy, dx);
        double heading_error = normalizeAngle(desired_yaw - pose->yaw);

        // Simple proportional controller
        double max_lin = 0.4;
        double max_ang = 1.0;

        double k_lin = 0.6;   // linear gain
        double k_ang = 2.0;   // angular gain

        if (dist < 0.1)
        {
            cmd.linear.x  = 0.0;
            cmd.angular.z = 0.0;
        }
        else
        {
            cmd.linear.x  = std::min(max_lin, k_lin * dist);
            cmd.angular.z = std::max(-max_ang,
                                     std::min(max_ang, k_ang * heading_error));
        }

        cmd_pub.publish(cmd);

        ROS_INFO_THROTTLE(1.0,
            "NavGoal: setpoint=%d, goal(%.2f,%.2f), pose(%.2f,%.2f,yaw=%.2f), "
            "dist=%.2f, cmd(v=%.2f,w=%.2f)",
            setpoint, goal->x, goal->y, pose->x, pose->y, pose->yaw,
            dist, cmd.linear.x, cmd.angular.z);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "main_cmdvel");
    ros::NodeHandle nh;

    GoalTracker goal(&nh);
    PoseTracker pose(&nh);
    NavController nav(&nh, &goal, &pose);

    ros::Rate rate(10);

    while (ros::ok())
    {
        nav.step();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
