// *************************************************************************************************
// Title        : obs_cmdvel.cpp (SIM / GAZEBO VERSION)
// Description  : Local obstacle avoidance. Subscribes to /scan and /cmd_vel_nav and publishes a
//                "safe" velocity on /cmd_vel for Gazebo diff drive.
//                IMPORTANT: if nav command is zero, we DO NOT override it (robot must stop).
// *************************************************************************************************

#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

// Helper: slice a vector [X..Y] inclusive
static std::vector<float> slicing(const std::vector<float>& arr, int X, int Y)
{
    auto start = arr.begin() + X;
    auto end   = arr.begin() + Y + 1;
    std::vector<float> result(Y - X + 1);
    std::copy(start, end, result.begin());
    return result;
}

class ObstacleAvoider
{
public:
    ObstacleAvoider(ros::NodeHandle* n)
    {
        int q = 10;
        scanSub_    = n->subscribe("scan", q, &ObstacleAvoider::laserScanCallback, this);
        navCmdSub_  = n->subscribe("cmd_vel_nav", q, &ObstacleAvoider::navCmdCallback, this);
        safeCmdPub_ = n->advertise<geometry_msgs::Twist>("cmd_vel", q);
    }

    void spin()
    {
        ros::Rate rate(10);   // 10 Hz

        while (ros::ok())
        {
            geometry_msgs::Twist cmd_out;

            if (!hasNavCmd_)
            {
                // No navigation command yet -> stop
                cmd_out.linear.x  = 0.0;
                cmd_out.angular.z = 0.0;
            }
            else
            {
                // Default: follow navigation command from sim_nav
                cmd_out = lastNavCmd_;

                // 🔹 NEW: detect if nav wants us to STOP
                bool navWantsStop =
                    std::fabs(lastNavCmd_.linear.x)  < 1e-3 &&
                    std::fabs(lastNavCmd_.angular.z) < 1e-3;

                // ----------------- OBSTACLE OVERRIDE -----------------
                // Only override if nav actually wants to move (NOT when stopping)
                if (!navWantsStop && frontObs_)
                {
                    // If we are EXTREMELY close, back up + turn to escape sticking
                    if (minFrontDist_ > 0.0 && minFrontDist_ < veryCloseDist_)
                    {
                        cmd_out.linear.x  = -0.2; // back up

                        // choose turn direction: if right blocked, turn left, else turn right
                        cmd_out.angular.z = (rightObs_ && !leftObs_) ?  turnSpeed_
                                                                     : -turnSpeed_;

                        ROS_WARN_THROTTLE(1.0,
                            "Obs: TOO CLOSE (%.2f m) -> BACKING UP & TURNING",
                            minFrontDist_);
                    }
                    else
                    {
                        // Normal avoidance (still forward, but turn strongly)
                        if (rightObs_ && !leftObs_)
                        {
                            // Front + right blocked -> turn LEFT
                            cmd_out.linear.x  = slowForward_;
                            cmd_out.angular.z =  turnSpeed_;
                            ROS_INFO_THROTTLE(1.0,
                                "Obs: front & right blocked -> turn LEFT");
                        }
                        else if (!rightObs_)
                        {
                            // Front blocked, right free -> turn RIGHT
                            cmd_out.linear.x  = slowForward_;
                            cmd_out.angular.z = -turnSpeed_;
                            ROS_INFO_THROTTLE(1.0,
                                "Obs: front blocked -> turn RIGHT");
                        }
                        else
                        {
                            // Both sides crowded -> U-turn-ish
                            cmd_out.linear.x  = slowForward_;
                            cmd_out.angular.z =  turnSpeed_;
                            ROS_INFO_THROTTLE(1.0,
                                "Obs: fully blocked -> U-TURN");
                        }
                    }
                }
                // ------------------------------------------------------
            }

            safeCmdPub_.publish(cmd_out);
            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    // Subscribers / publisher
    ros::Subscriber scanSub_;
    ros::Subscriber navCmdSub_;
    ros::Publisher  safeCmdPub_;

    // State from /scan
    bool   leftObs_{false};
    bool   frontObs_{false};
    bool   rightObs_{false};
    double minFrontDist_{0.0};   // closest obstacle in front (meters)

    // State from /cmd_vel_nav
    geometry_msgs::Twist lastNavCmd_;
    bool hasNavCmd_{false};

    // Tunable parameters
    double obsDistThresh_{0.7};   // meters: when we start counting an obstacle
    int    pixelThresh_{20};      // min # of points to count as obstacle
    double slowForward_{0.02};    // m/s when avoiding
    double turnSpeed_{0.9};       // rad/s turn speed when avoiding
    double veryCloseDist_{0.25};  // m: if closer than this, back up

    void navCmdCallback(const geometry_msgs::Twist::ConstPtr& msg)
    {
        lastNavCmd_ = *msg;
        hasNavCmd_  = true;
    }

    void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& data)
    {
        const std::vector<float>& ranges = data->ranges;

        if (ranges.size() < 480)
        {
            ROS_WARN_THROTTLE(2.0, "Obs: scan size (%zu) < 480, skipping",
                              ranges.size());
            return;
        }

        // Split into left (0-159), front (160-319), right (320-479)
        std::vector<float> left  = slicing(ranges, 0,   159);
        std::vector<float> front = slicing(ranges, 160, 319);
        std::vector<float> right = slicing(ranges, 320, 479);

        int left_count  = 0;
        int front_count = 0;
        int right_count = 0;

        // Track minimum front distance
        minFrontDist_ = 0.0;
        bool firstFront = true;

        for (float d : left)
        {
            if (d != 0.0 && d < obsDistThresh_) left_count++;
        }

        for (float d : front)
        {
            if (d != 0.0 && d < obsDistThresh_) front_count++;

            if (d != 0.0)
            {
                if (firstFront)
                {
                    minFrontDist_ = d;
                    firstFront = false;
                }
                else if (d < minFrontDist_)
                {
                    minFrontDist_ = d;
                }
            }
        }

        for (float d : right)
        {
            if (d != 0.0 && d < obsDistThresh_) right_count++;
        }

        leftObs_  = (left_count  > pixelThresh_);
        frontObs_ = (front_count > pixelThresh_);
        rightObs_ = (right_count > pixelThresh_);

        ROS_INFO_THROTTLE(1.0,
            "Obs flags -> L:%d F:%d R:%d (counts %d,%d,%d) minFront=%.2f",
            (int)leftObs_, (int)frontObs_, (int)rightObs_,
            left_count, front_count, right_count, minFrontDist_);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "obs_cmdvel");
    ros::NodeHandle nh;

    ObstacleAvoider obs(&nh);
    obs.spin();

    return 0;
}
