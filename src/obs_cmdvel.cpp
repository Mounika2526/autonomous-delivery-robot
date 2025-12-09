// *************************************************************************************************
// Title        : obs_cmdvel.cpp (SIM / GAZEBO VERSION)
// Description  : Local obstacle avoidance. Subscribes to /scan and /cmd_vel_nav and publishes a
//                "safe" velocity on /cmd_vel for Gazebo diff drive.
//                Logic (simple & robust):
//                  - If front is CLEAR  -> forward = exactly /cmd_vel_nav
//                  - If front is BLOCKED -> ignore /cmd_vel_nav, rotate in place to avoid
// *************************************************************************************************

#include <iostream>
#include <vector>
#include <algorithm>

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
        scanSub_    = n->subscribe("scan",       q, &ObstacleAvoider::laserScanCallback, this);
        navCmdSub_  = n->subscribe("cmd_vel_nav",q, &ObstacleAvoider::navCmdCallback,   this);
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
                // ======= DEFAULT: follow navigation command ==========
                cmd_out = lastNavCmd_;

                // ======= OBSTACLE OVERRIDE ===========================
                if (frontObs_)
                {
                    // Ignore the nav command completely, rotate in place.
                    cmd_out.linear.x = 0.0;

                    // If right is blocked (and left is clearer), turn LEFT,
                    // otherwise turn RIGHT. This tends to move us into free space.
                    if (rightObs_ && !leftObs_)
                    {
                        cmd_out.angular.z =  turnSpeed_;
                        ROS_WARN_THROTTLE(1.0,
                            "Obs: FRONT & RIGHT blocked -> rotate LEFT");
                    }
                    else
                    {
                        cmd_out.angular.z = -turnSpeed_;
                        ROS_WARN_THROTTLE(1.0,
                            "Obs: FRONT blocked -> rotate RIGHT");
                    }
                }
                // If only side obstacles exist (no front obstacle), we simply keep
                // following /cmd_vel_nav. (Your global nav is already steering.)
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
    bool leftObs_{false};
    bool frontObs_{false};
    bool rightObs_{false};

    // State from /cmd_vel_nav
    geometry_msgs::Twist lastNavCmd_;
    bool hasNavCmd_{false};

    // Tunable parameters
    double obsDistThresh_{0.7};   // meters: obstacle if closer than this
    int    pixelThresh_{25};      // min # of points to count as obstacle
    double turnSpeed_{0.7};       // rad/s turn speed when front blocked

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

        for (float d : left)
            if (d > 0.0 && d < obsDistThresh_) left_count++;

        for (float d : front)
            if (d > 0.0 && d < obsDistThresh_) front_count++;

        for (float d : right)
            if (d > 0.0 && d < obsDistThresh_) right_count++;

        leftObs_  = (left_count  > pixelThresh_);
        frontObs_ = (front_count > pixelThresh_);
        rightObs_ = (right_count > pixelThresh_);

        ROS_INFO_THROTTLE(1.0,
            "Obs flags -> L:%d F:%d R:%d (counts %d,%d,%d)",
            (int)leftObs_, (int)frontObs_, (int)rightObs_,
            left_count, front_count, right_count);
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
