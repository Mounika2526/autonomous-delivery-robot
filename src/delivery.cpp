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

    SetPoint(ros::NodeHandle* n)
    {
        int queue_size = 1000;
        m_setpointPub = n->advertise<std_msgs::Int8>("setpoint", queue_size);
        m_reachedSetpointSub = n->subscribe("isReachedSetPoint", queue_size, &SetPoint::reached_setpoint_callback, this);
    }

    void publish_setpoint(int sp)
    {
        std_msgs::Int8 value;
        value.data = sp;
        m_setpointPub.publish(value);
    }

    void reached_setpoint_callback(const std_msgs::Bool::ConstPtr &value)
    {
        m_isReachedSetPoint = value->data;
    }

    bool find_setpoint(std::string location)
    {
        if (location.compare("Location A") == 0)
        {
            publish_setpoint(0);
        }
        else if (location.compare("Location B") == 0)
        {
            publish_setpoint(1);
        }
        else
        {
            publish_setpoint(-1);
            return 0;
        }

        return 1;
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
            // Pickup point (example coords)
            p.x = 0.5;
            p.y = 0.5;
            p.z = 0.0;
        }
        else if (location == "Location B")
        {
            // Dropoff point (example coords)
            p.x = 3.0;
            p.y = 2.0;
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
    ProgressStatus progressStatus{&n};
    SetPoint setpoint{&n};
    Sender sender{&n};
    Receiver receiver{&n};
    GoalPublisher goalPublisher{&n};   // NEW

    ros::Rate loopRate(10);

    time_t loopStartTime;
    time_t loopCurrTime;
    time_t timeDiff{0};

    std::cout << '\n';

    while (ros::ok())
    {
        time_t timeDiff{0};
        int senderCount{0};

        // ---------------- SENDER LOOP ----------------
        // Simulate navigating to sender's pickup location.
        while (ros::ok())
        {
            bool isfoundSetpoint{setpoint.find_setpoint(sender.m_location)};

            if (isfoundSetpoint)
            {
                if (senderCount == 0)
                {
                    time(&loopStartTime);
                    senderCount++;
                }

                // Keep telling nav_sim where to go for pickup:
                // e.g. "Location A" -> (0.5,0.5)
                goalPublisher.publish_goal_for_location(sender.m_location);

                time(&loopCurrTime);
                timeDiff = loopCurrTime - loopStartTime;

                ROS_INFO("Found setpoint");
                botAvailability.set_availability_status(AvailabilityStatusOptions::no);
                progressStatus.set_progress_status(ProgressStatusOptions::in_progress);
            }

            // Allow topics to update
            sleep(2);
            ros::spinOnce();
            loopRate.sleep();

            // After ~6 seconds, assume we've reached sender
            if (timeDiff > 6)
            {
                ROS_INFO("Reached sender at %s", sender.m_location.c_str());
                break;
            }
        }

        // ---------------- RECEIVER LOOP ----------------
        // Now simulate navigating to receiver/dropoff.
        time(&loopStartTime);

        while (ros::ok())
        {
            int receiverCount{0}; // currently unused but kept for parity

            bool isfoundSetpoint{setpoint.find_setpoint(receiver.m_location)};
            (void)isfoundSetpoint;

            // Keep telling nav_sim where to go for dropoff:
            // e.g. "Location B" -> (3.0,2.0)
            goalPublisher.publish_goal_for_location(receiver.m_location);

            sleep(1);
            ros::spinOnce();
            loopRate.sleep();

            time(&loopCurrTime);
            timeDiff = loopCurrTime - loopStartTime;

            // After ~6 seconds, assume we've reached receiver
            if (timeDiff > 6)
            {
                progressStatus.set_progress_status(ProgressStatusOptions::done);
                botAvailability.set_availability_status(AvailabilityStatusOptions::yes);

                ROS_INFO("Reached receiver at %s", receiver.m_location.c_str());
                ROS_INFO("Completed delivery!");

                ros::spinOnce();
                loopRate.sleep();
                sleep(1);
                break;
            }
        }

        std::cout << '\n';

        // Small pause before restarting another delivery cycle
        sleep(4);
        ros::spinOnce();
        loopRate.sleep();
    }
}
