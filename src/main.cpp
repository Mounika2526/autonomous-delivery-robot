// ****************************************************************************************************************************************
// Title            : main.cpp (SIM BUILD VERSION - no GPIO)
// Description      : Sim version for Ubuntu VM. Navigates to setpoint logically but does NOT touch Raspberry Pi pins.
// ****************************************************************************************************************************************

#include <cmath>
#include <ctime>
#include <cstdlib>
#include <iostream>
#include <string>
#include <unistd.h>            // for sleep/usleep if needed

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int8.h"

// ---------------------- motorControl ----------------------
// In the real robot, this would configure GPIO pins, PWM, etc.
// In the VM build, we just store the pin numbers so code compiles.
class motorControl
{
public:
    int m_dirPin;
    int m_pwmPin;
    int m_speed;

    motorControl(int dirPin, int pwmPin)
    {
        m_dirPin = dirPin;
        m_pwmPin = pwmPin;
        m_speed  = 0;

        // REAL ROBOT VERSION (commented out for VM):
        // pinMode(m_dirPin, OUTPUT);
        // pinMode(m_pwmPin, OUTPUT);
        // softPwmCreate(m_pwmPin, 0, 100);
    }
};

// ---------------------- SetPoint ----------------------
// Listens for "setpoint" Int8 messages and publishes "isReachedSetPoint" Bool.
class SetPoint
{
public:
    ros::Publisher m_reachedSetpointPub{};
    ros::Subscriber m_setpointSub{};
    int m_setpoint{-1};
    
    SetPoint(ros::NodeHandle* n)
    {
        int queue_size = 1000;
        m_reachedSetpointPub = n->advertise<std_msgs::Bool>("isReachedSetPoint", queue_size);
        m_setpointSub = n->subscribe("setpoint", 1, &SetPoint::setpoint_callback, this);
    }

    void setpoint_callback(const std_msgs::Int8::ConstPtr &val)
    {
        m_setpoint = val->data;
    }

    void reachedSetpointStatus(int boolVal)
    {
        std_msgs::Bool value;
        value.data = boolVal;
        m_reachedSetpointPub.publish(value);
    }
};

// ---------------------- Navigation ----------------------
// Handles velocities and "movement logic". In VM we just print instead of driving motors.
class Navigation
{
public:
    const int m_maxSpeed {15};
    int m_rightVelocity{0};
    int m_leftVelocity{0};

    motorControl* m_fwRight;
    motorControl* m_fwLeft;
    motorControl* m_bwRight;
    motorControl* m_bwLeft;

    Navigation(ros::NodeHandle* /*n*/, motorControl* fwRight, motorControl* fwLeft, motorControl* bwRight, motorControl* bwLeft)
    {
        m_fwRight = fwRight;
        m_fwLeft  = fwLeft;
        m_bwRight = bwRight;
        m_bwLeft  = bwLeft;
    }

    void navigate_to_point(SetPoint* setpoint)
    {
        // Reset to stopped each cycle, then set based on setpoint
        m_rightVelocity = 0;
        m_leftVelocity  = 0;

        // Location A: straight forward
        if (setpoint->m_setpoint == 0)
        {
            m_rightVelocity = 15;
            m_leftVelocity  = 15;
            ROS_INFO("Nav: Location A (setpoint 0) -> straight forward");
        }
        // Location B: straight backward
        else if (setpoint->m_setpoint == 1)
        {
            m_rightVelocity = -15;
            m_leftVelocity  = -15;
            ROS_INFO("Nav: Location B (setpoint 1) -> straight backward");
        }
        // Location C: forward with slight right arc
        else if (setpoint->m_setpoint == 2)
        {
            m_rightVelocity = 10;   // slower right
            m_leftVelocity  = 15;   // faster left  -> arc to the right
            ROS_INFO("Nav: Location C (setpoint 2) -> forward + slight RIGHT turn");
        }
        // Location D: forward with slight left arc
        else if (setpoint->m_setpoint == 3)
        {
            m_rightVelocity = 15;   // faster right
            m_leftVelocity  = 10;   // slower left  -> arc to the left
            ROS_INFO("Nav: Location D (setpoint 3) -> forward + slight LEFT turn");
        }
        else
        {
            // Unknown / unset setpoint
            ROS_WARN("Nav: unknown setpoint %d, stopping.", setpoint->m_setpoint);
        }

        set_velocity();
    }


    // This function was broken before because half of it was commented out.
    // Here we turn it into a safe stub that just logs.
    void set_velocity()
    {
        ROS_INFO("set_velocity -> R:%d  L:%d", m_rightVelocity, m_leftVelocity);

        // REAL ROBOT VERSION (commented out):
        // if (m_rightVelocity >= 0) {
        //     digitalWrite(m_fwRight->m_dirPin, HIGH);
        //     digitalWrite(m_fwLeft->m_dirPin, HIGH);
        //     digitalWrite(m_bwRight->m_dirPin, HIGH);
        //     digitalWrite(m_bwLeft->m_dirPin, HIGH);
        // } else {
        //     digitalWrite(m_fwRight->m_dirPin, LOW);
        //     digitalWrite(m_fwLeft->m_dirPin, LOW);
        //     digitalWrite(m_bwRight->m_dirPin, LOW);
        //     digitalWrite(m_bwLeft->m_dirPin, LOW);
        // }

        // softPwmWrite(m_fwRight->m_pwmPin, abs(m_rightVelocity));
        // softPwmWrite(m_fwLeft->m_pwmPin,  abs(m_rightVelocity));
        // softPwmWrite(m_bwRight->m_pwmPin, abs(m_rightVelocity));
        // softPwmWrite(m_bwLeft->m_pwmPin,  abs(m_rightVelocity));
    }

    void stopMotors()
    {
        // VM version: just zero them out and log
        m_rightVelocity = 0;
        m_leftVelocity  = 0;

        ROS_INFO("stopMotors -> R:%d  L:%d", m_rightVelocity, m_leftVelocity);

        // REAL ROBOT VERSION (commented out):
        // while(m_rightVelocity >= 0)
        // {
        //     softPwmWrite(m_fwRight->m_pwmPin, m_rightVelocity);
        //     softPwmWrite(m_fwLeft->m_pwmPin,  m_rightVelocity);
        //     softPwmWrite(m_bwRight->m_pwmPin, m_rightVelocity);
        //     softPwmWrite(m_bwLeft->m_pwmPin,  m_rightVelocity);
        //     m_rightVelocity--;
        // }
        //
        // digitalWrite(m_fwRight->m_pwmPin, LOW);
        // digitalWrite(m_fwLeft->m_pwmPin, LOW);
        // digitalWrite(m_bwRight->m_pwmPin, LOW);
        // digitalWrite(m_bwLeft->m_pwmPin, LOW);
        //
        // delay(3);
    }

    // Utility math function is unchanged
    double find_distance_between_points(double point1[3], double point2[3])
    {
        return std::sqrt(
            std::pow((point1[0]-point2[0]), 2) +
            std::pow((point1[2]-point2[2]), 2)
        );
    }
};

// ---------------------- main() ----------------------
int main(int argc, char **argv)
{
    ros::init(argc, argv, "main");
    ros::NodeHandle n;

    // REAL ROBOT VERSION (commented out):
    // if(wiringPiSetup()<0) { std::cout<<"setup wiring pi failed\n"; return 1; }

    // Create motor objects. On Pi, these would map to actual GPIO pins.
    motorControl fwRight{5, 4};
    motorControl fwLeft{3, 2};
    motorControl bwRight{29, 28};
    motorControl bwLeft{25, 24};

    Navigation navigation{&n, &fwRight, &fwLeft, &bwRight, &bwLeft};
    SetPoint setpoint(&n);

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        if (setpoint.m_setpoint != -1)  // setpoint received
        {
            navigation.navigate_to_point(&setpoint);
            ROS_INFO("Navigating toward setpoint %d", setpoint.m_setpoint);

            // OPTIONAL FUTURE: setpoint.reachedSetpointStatus(1) when reached.
        }
        else
        {
            navigation.stopMotors();
            setpoint.reachedSetpointStatus(0);
        }
        
        ros::spinOnce();
        loop_rate.sleep();
    }   

    return 0;
}
