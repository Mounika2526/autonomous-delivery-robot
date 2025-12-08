// *************************************************************************************************
// Title        : main_cmdvel.cpp  (SIM / GAZEBO VERSION)
// Description  : Subscribes to "setpoint" (Int8) and publishes "cmd_vel" (geometry_msgs/Twist)
//                for a differential drive robot in Gazebo.
// *************************************************************************************************

#include <iostream>
#include <string>

#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "geometry_msgs/Twist.h"

// ---------------------- SetPoint ----------------------
class SetPoint
{
public:
    ros::Subscriber m_setpointSub{};
    int m_setpoint{-1};

    SetPoint(ros::NodeHandle* n)
    {
        int queue_size = 10;
        m_setpointSub = n->subscribe("setpoint", queue_size,
                                     &SetPoint::setpoint_callback, this);
    }

    void setpoint_callback(const std_msgs::Int8::ConstPtr &val)
    {
        m_setpoint = val->data;
        ROS_INFO("SetPoint: received setpoint %d", m_setpoint);
    }
};

// ---------------------- NavigationCmdVel ----------------------
class NavigationCmdVel
{
public:
    ros::Publisher m_cmdVelPub{};

    // Base speed parameters (tune these for your sim)
    double m_linSpeed{0.3};   // m/s
    double m_angSpeed{0.6};   // rad/s

    NavigationCmdVel(ros::NodeHandle* n)
    {
        int queue_size = 10;
        m_cmdVelPub = n->advertise<geometry_msgs::Twist>("cmd_vel_nav", queue_size);
    }

    void navigate_to_point(const SetPoint* setpoint)
    {
        geometry_msgs::Twist cmd;

        // Default: stop
        cmd.linear.x  = 0.0;
        cmd.angular.z = 0.0;

        if (setpoint->m_setpoint == 0)                  // Location A
        {
            // straight forward
            cmd.linear.x  = m_linSpeed;
            cmd.angular.z = 0.0;
            ROS_INFO_THROTTLE(1.0, "Nav: A (0) -> forward");
        }
        else if (setpoint->m_setpoint == 1)             // Location B
        {
            // straight backward
            cmd.linear.x  = -m_linSpeed;
            cmd.angular.z = 0.0;
            ROS_INFO_THROTTLE(1.0, "Nav: B (1) -> backward");
        }
        else if (setpoint->m_setpoint == 2)             // Location C
        {
            // forward + slight RIGHT arc
            cmd.linear.x  = m_linSpeed;
            cmd.angular.z = -m_angSpeed;
            ROS_INFO_THROTTLE(1.0, "Nav: C (2) -> forward + RIGHT");
        }
        else if (setpoint->m_setpoint == 3)             // Location D
        {
            // forward + slight LEFT arc
            cmd.linear.x  = m_linSpeed;
            cmd.angular.z = m_angSpeed;
            ROS_INFO_THROTTLE(1.0, "Nav: D (3) -> forward + LEFT");
        }
        else
        {
            ROS_WARN_THROTTLE(2.0, "Nav: unknown/unset setpoint (%d), stopping.",
                               setpoint->m_setpoint);
        }

        m_cmdVelPub.publish(cmd);
    }
};

// ---------------------- main() ----------------------
int main(int argc, char **argv)
{
    ros::init(argc, argv, "main_cmdvel");
    ros::NodeHandle n;

    SetPoint setpoint(&n);
    NavigationCmdVel nav(&n);

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        nav.navigate_to_point(&setpoint);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
