// ****************************************************************************************************************************************
// Title            : delivery.cpp
// Description      : The script is responsible for resolving the sender and receiver information and determines the location setpoint and
//                    publishes it. It also publishes appropriate status signals.
//                    (SIM VERSION) Additionally, this version publishes a 2D navigation goal for the RViz navigation simulator.
// Author           : Sowbhagya Lakshmi H T
// Last revised on  : 20/05/2023
// NOTE (SIM BUILD VERSION - no GPIO): wiringPi removed for Ubuntu VM
// ****************************************************************************************************************************************

#include <array>
#include <iostream>
#include <string>
#include <vector>
#include <ctime>
#include <csignal>
#include <sstream>

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int8.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Point.h"   // NEW: publish nav goal positions
#include <nav_msgs/Odometry.h>
#include <cmath>

// #include <wiringPi.h>   // REMOVED FOR VM
// #include <softPwm.h>    // REMOVED FOR VM

// For sleep function
#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif

// --------------------------------------------------------------------------------------
// ENUMS (unchanged)
// --------------------------------------------------------------------------------------

enum class ProgressStatusOptions
{
    done,
    in_progress,
    none,
};

enum class AvailabilityStatusOptions
{
    yes,
    no,
};

// Returns true if we have coordinates for this logical location
bool get_location_xy(const std::string& location, double& x, double& y)
{
    if (location == "Location C")
    {
        x = 0.5;  // from <pose>0.5 0.0 0.2 ...</pose> for location_C in world
        y = -0.8;
        return true;
    }
    else if (location == "Location D")
    {
        x = 3.0;  // from <pose>3.0 0.0 0.2 ...</pose> for location_D in world
        y = 0.8;
        return true;
    }
    // Optional: add A/B here if you want them too.
    // else if (location == "Location A") { ... }

    return false;
}


// Tracks robot pose from /odom (x,y)
class RobotPose
{
public:
    ros::Subscriber m_odomSub{};
    double m_x{0.0};
    double m_y{0.0};
    bool   m_hasPose{false};

    RobotPose(ros::NodeHandle* n)
    {
        int queue_size = 50;
        m_odomSub = n->subscribe("odom", queue_size,
                                 &RobotPose::odom_callback, this);
    }

    void odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        m_x = msg->pose.pose.position.x;
        m_y = msg->pose.pose.position.y;
        m_hasPose = true;
    }
};

// --------------------------------------------------------------------------------------
// SENDER: listens to "senderLocation" (string, e.g. "Location A")
// --------------------------------------------------------------------------------------

class Sender
{
public:
    std::string m_location{};
    ros::Subscriber m_locationSub{};

    Sender(ros::NodeHandle* n)
    {
        int queue_size = 1000;
        m_locationSub = n->subscribe("senderLocation", queue_size, &Sender::location_callback, this);
    }

    void location_callback(const std_msgs::String::ConstPtr &name)
    {
        this->m_location = name->data.c_str();
    }
};

// --------------------------------------------------------------------------------------
// RECEIVER: listens to "receiverLocation" (string, e.g. "Location B")
// --------------------------------------------------------------------------------------

class Receiver
{
public:
    std::string m_location{};
    ros::Subscriber m_locationSub{};

    Receiver(ros::NodeHandle* n)
    {
        int queue_size = 1000;
        m_locationSub = n->subscribe("receiverLocation", queue_size, &Receiver::location_callback, this);
    }

    void location_callback(const std_msgs::String::ConstPtr &name)
    {
        m_location = name->data.c_str();
    }
};

// --------------------------------------------------------------------------------------
// SETPOINT: publishes which logical site (0=A, 1=B) on "setpoint", subscribes to "isReachedSetPoint"
// --------------------------------------------------------------------------------------

class SetPoint
{
public:
    bool m_isReachedSetPoint{0};  
    ros::Publisher m_setpointPub{};
    ros::Subscriber m_reachedSetpointSub{};
    int m_setpoint{-1};

    // 🔹 NEW: table of all supported locations
    std::vector<std::string> m_locations{
        "Location A",
        "Location B",
        "Location C",
        "Location D"
        // You can add more: "Location E", ...
    };

    SetPoint(ros::NodeHandle* n)
    {
        int queue_size = 1000;
        m_setpointPub = n->advertise<std_msgs::Int8>("setpoint", queue_size);
        m_reachedSetpointSub = n->subscribe(
            "isReachedSetPoint", queue_size,
            &SetPoint::reached_setpoint_callback, this);
    }

    void publish_setpoint(int sp)
    {
        std_msgs::Int8 value;
        value.data = sp;
        m_setpointPub.publish(value);

        // Optional: keep internal copy of last setpoint
        m_setpoint = sp;
    }

    void reached_setpoint_callback(const std_msgs::Bool::ConstPtr &value)
    {   
        m_isReachedSetPoint = value->data;
    }

    // 🔹 CHANGED: generic mapping from location string → setpoint index
    bool find_setpoint(const std::string &location)
    {
        int setpoint_index = -1;

        // Look up the location in our table
        for (std::size_t i = 0; i < m_locations.size(); ++i)
        {
            if (location == m_locations[i])
            {
                setpoint_index = static_cast<int>(i);
                break;
            }
        }

        if (setpoint_index == -1)
        {
            // Unknown location
            publish_setpoint(-1);
            ROS_WARN("SetPoint::find_setpoint() - Unknown location: '%s'",
                     location.c_str());
            return false;
        }

        publish_setpoint(setpoint_index);
        ROS_INFO("SetPoint::find_setpoint() - Location '%s' mapped to setpoint %d",
                 location.c_str(), setpoint_index);
        return true;
    }
};

// --------------------------------------------------------------------------------------
// AVAILABILITY publisher ("availability" topic)
// --------------------------------------------------------------------------------------

class BotAvailability
{
public:
    ros::Publisher m_availabilityPub{};
    std::vector<std::string> m_availabilityStatusOptions{"yes", "no"};

    BotAvailability(ros::NodeHandle* n)
    {
        int queue_size = 1000;
        m_availabilityPub = n->advertise<std_msgs::String>("availability", queue_size);
    }

    void set_availability_status(AvailabilityStatusOptions status)
    {
        std::stringstream strStreamAvailabilityStatus;
        strStreamAvailabilityStatus << m_availabilityStatusOptions[static_cast<int>(status)];

        std_msgs::String availabilityStatus;
        availabilityStatus.data = strStreamAvailabilityStatus.str();

        m_availabilityPub.publish(availabilityStatus);
    }
};

// --------------------------------------------------------------------------------------
// PROGRESS publisher ("progress" topic)
// --------------------------------------------------------------------------------------

class ProgressStatus
{
public:
    ros::Publisher m_progressStatusPub{};
    std::vector<std::string> m_progressStatusOptions{"done", "in progress", "none"};

    ProgressStatus(ros::NodeHandle* n)
    {
        int queue_size = 1000;
        m_progressStatusPub = n->advertise<std_msgs::String>("progress", queue_size);
    }

    void set_progress_status(ProgressStatusOptions status)
    {
        std::stringstream strStreamProgressStatus;
        strStreamProgressStatus << m_progressStatusOptions[static_cast<int>(status)];

        std_msgs::String progressStatus;
        progressStatus.data = strStreamProgressStatus.str();

        m_progressStatusPub.publish(progressStatus);
    }
};

// --------------------------------------------------------------------------------------
// GOAL publisher for nav_sim (NEW)
// This publishes a 2D coordinate (x,y) for the navigation simulator in RViz.
// Topic: "delivery_goal" (geometry_msgs/Point)
// --------------------------------------------------------------------------------------

class GoalPublisher
{
public:
    ros::Publisher m_goalPub;

    GoalPublisher(ros::NodeHandle* n)
    {
        int queue_size = 10;
        m_goalPub = n->advertise<geometry_msgs::Point>("delivery_goal", queue_size);
    }

    // Map "Location A" / "Location B" -> world coordinates in meters.
    // You can tune these so they line up with what you see in RViz.
    void publish_goal_for_location(const std::string& location)
    {
        geometry_msgs::Point p;

        if (location == "Location A")
        {
            // Example pickup point in nav_sim map
            p.x = 0.5;
            p.y = 0.5;
            p.z = 0.0;
        }
        else if (location == "Location B")
        {
            // Example dropoff point in nav_sim map
            p.x = 3.0;
            p.y = 2.0;
            p.z = 0.0;
        }
        else if (location == "Location C")
        {
            // Match red cube in Gazebo (location_C pose)
            p.x = 0.5;
            p.y = 0.0;
            p.z = 0.0;
        }
        else if (location == "Location D")
        {
            // Match blue cube in Gazebo (location_D pose)
            p.x = 3.0;
            p.y = 0.0;
            p.z = 0.0;
        }
        else
        {
            // Unknown / idle (default to origin)
            p.x = 0.0;
            p.y = 0.0;
            p.z = 0.0;
        }

        m_goalPub.publish(p);
    }
};

// --------------------------------------------------------------------------------------
// MAIN
// --------------------------------------------------------------------------------------

int main(int argc, char **argv)
{
    ros::init(argc, argv, "delivery");
    ros::NodeHandle n;

    BotAvailability botAvailability{&n};
    ProgressStatus  progressStatus{&n};
    SetPoint        setpoint{&n};
    Sender          sender{&n};
    Receiver        receiver{&n};
    RobotPose       robotPose{&n};
    GoalPublisher   goalPublisher{&n};

    ros::Rate loopRate(10);

    std::cout << '\n';

    // Store last completed order, so we don't immediately repeat it
    std::string last_sender;
    std::string last_receiver;

    while (ros::ok())
    {
        // ============= 0. WAIT FOR A NEW ORDER =============
        ROS_INFO("Delivery: waiting for a new sender/receiver order...");

        // Wait until both sender and receiver are non-empty AND different from last completed
        while (ros::ok())
        {
            ros::spinOnce();

            bool has_sender   = !sender.m_location.empty();
            bool has_receiver = !receiver.m_location.empty();
            bool is_new_order =
                (sender.m_location != last_sender) ||
                (receiver.m_location != last_receiver);

            if (has_sender && has_receiver && is_new_order)
            {
                ROS_INFO("New order received: sender='%s', receiver='%s'",
                         sender.m_location.c_str(), receiver.m_location.c_str());
                break;
            }

            ROS_INFO_THROTTLE(5.0,
                "Waiting... sender='%s', receiver='%s', last_sender='%s', last_receiver='%s'",
                sender.m_location.c_str(), receiver.m_location.c_str(),
                last_sender.c_str(), last_receiver.c_str());

            loopRate.sleep();
        }

        if (!ros::ok())
            break;

        // Mark that bot is now busy
        botAvailability.set_availability_status(AvailabilityStatusOptions::no);
        progressStatus.set_progress_status(ProgressStatusOptions::in_progress);

        // ============= 1. SENDER LOOP =============
        while (ros::ok())
        {
            bool isfoundSetpoint = setpoint.find_setpoint(sender.m_location);

            if (isfoundSetpoint)
            {
                ROS_INFO("Found setpoint for sender: %s", sender.m_location.c_str());
                goalPublisher.publish_goal_for_location(sender.m_location);
            }

            // Only check distance if we have pose + target coords
            double target_x, target_y;
            if (robotPose.m_hasPose &&
                get_location_xy(sender.m_location, target_x, target_y))
            {
                double dx = robotPose.m_x - target_x;
                double dy = robotPose.m_y - target_y;
                double dist = std::sqrt(dx * dx + dy * dy);

                double reach_thresh = 0.5; // tune as needed

                if (dist < reach_thresh)
                {
                    ROS_INFO("Reached sender at %s (distance %.2f m)",
                             sender.m_location.c_str(), dist);
                    break;
                }
            }

            sleep(1);
            ros::spinOnce();
            loopRate.sleep();
        }

        if (!ros::ok())
            break;

        // ============= 2. RECEIVER LOOP =============
        while (ros::ok())
        {
            bool isfoundSetpoint = setpoint.find_setpoint(receiver.m_location);

            if (isfoundSetpoint)
            {
                ROS_INFO("Found setpoint for receiver: %s", receiver.m_location.c_str());
                goalPublisher.publish_goal_for_location(receiver.m_location);
            }

            double target_x, target_y;
            if (robotPose.m_hasPose &&
                get_location_xy(receiver.m_location, target_x, target_y))
            {
                double dx = robotPose.m_x - target_x;
                double dy = robotPose.m_y - target_y;
                double dist = std::sqrt(dx * dx + dy * dy);

                double reach_thresh = 0.8;

                if (dist < reach_thresh)
                {
                    progressStatus.set_progress_status(ProgressStatusOptions::done);
                    botAvailability.set_availability_status(AvailabilityStatusOptions::yes);

                    // Tell navigation to stop
                    setpoint.publish_setpoint(-1);

                    ROS_INFO("Reached receiver at %s (distance %.2f m)",
                             receiver.m_location.c_str(), dist);
                    ROS_INFO("Completed delivery!");
                    break;
                }
            }

            sleep(1);
            ros::spinOnce();
            loopRate.sleep();
        }

        if (!ros::ok())
            break;

        // ============= 3. RECORD COMPLETED ORDER & RESET =============
        last_sender   = sender.m_location;
        last_receiver = receiver.m_location;

        // Optional: clear locations so we don't reuse by accident
        // (If your database/app will publish a fresh order each time,
        // you can comment these out if you like.)
        sender.m_location.clear();
        receiver.m_location.clear();

        ROS_INFO("Delivery: cycle finished for sender='%s', receiver='%s'. "
                 "Waiting for next order...",
                 last_sender.c_str(), last_receiver.c_str());

        // Short pause before listening for next order
        sleep(2);
        ros::spinOnce();
        loopRate.sleep();
    }

    ROS_INFO("Delivery node shutting down.");
    return 0;
}
