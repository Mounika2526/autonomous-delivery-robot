// ****************************************************************************************************************************************
// Title            : obs_main.cpp
// Description      : The script is responsible for local obstacle avoidance. It subscribes to depth information in the form of LaserScan
//                    data and calculates the possibility and positioning of obstacles, then generates appropriate commands for the robot
//                    to dodge the obstacle.
// Author           : Sowbhagya Lakshmi H T
// Last revised on  : 20/05/2023
// NOTE (SIM BUILD VERSION - no GPIO): this version is patched to compile on Ubuntu VM without wiringPi.
// ****************************************************************************************************************************************

#include <iostream>
#include "ros/ros.h"
#include <sensor_msgs/LaserScan.h>
#include <vector>
#include <cmath>
#include <cstdlib>
#include <math.h>
#include <string>
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Imu.h>
// #include <wiringPi.h>   // <<< COMMENTED OUT FOR VM >>>
#include <ctime>
#include <csignal>
// #include <softPwm.h>    // <<< COMMENTED OUT FOR VM >>>

// >>> CHANGE HERE <<<
// We are running on plain Ubuntu with no Raspberry Pi GPIO,
// so we create "stub" functions and constants to satisfy the compiler.
#include <unistd.h>  // for usleep in our delay() stub

static inline void pinMode(int /*pin*/, int /*mode*/) {
    // no-op in simulation
}

static inline void digitalWrite(int /*pin*/, int /*val*/) {
    // no-op in simulation
}

static inline int softPwmCreate(int /*pin*/, int /*initVal*/, int /*range*/) {
    // pretend success
    return 0;
}

static inline void softPwmWrite(int /*pin*/, int /*val*/) {
    // no-op in simulation
}

static inline void delay(int ms) {
    // wiringPi delay() is milliseconds; usleep is microseconds
    usleep(ms * 1000);
}

static inline int wiringPiSetup() {
    // pretend setup is fine
    return 0;
}

// wiringPi usually gives you these:
static const int HIGH = 1;
static const int LOW  = 0;
static const int OUTPUT = 1;
// <<< END OF STUB BLOCK >>>


int flag = 0; 

// flag = 0 -> Go Forward
// flag = 1 -> right
// flag = 2 -> left
// flag = 3 -> u turn

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
        m_speed = 0;

        // Set pin mode (REAL ROBOT ONLY)
        // pinMode(m_dirPin, OUTPUT);
        // pinMode(m_pwmPin, OUTPUT);

        // Create soft pwm (REAL ROBOT ONLY)
        // int pwmInitialVal = 0;
        // int pwmRange = 100;
        // softPwmCreate(m_pwmPin, pwmInitialVal, pwmRange);
    }
};

class Navigation
{
public:
    const int m_maxSpeed {30};
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

        // REAL ROBOT VERSION:
        // digitalWrite(m_fwRight->m_dirPin, HIGH);
        // digitalWrite(m_fwLeft->m_dirPin, HIGH);
        // digitalWrite(m_bwRight->m_dirPin, HIGH);
        // digitalWrite(m_bwLeft->m_dirPin, HIGH);
    }

    void navigate_to_point()
    {
        if (flag == 0)
        {
            // go forward
            m_rightVelocity = 30;
            m_leftVelocity  = 30;
        }
        else if (flag == 1)     // right
        {
            m_rightVelocity = 5;
            m_leftVelocity  = 40;
        }
        else if (flag == 2)     // left
        {
            m_rightVelocity = 40;
            m_leftVelocity  = 5;
        }
        else if (flag == 3)     // U turn / stop
        {
            m_rightVelocity = 0;
            m_leftVelocity  = 0;
        }
        else
        {
            std::cout << "Unexpected\n";
        }

        set_velocity();
    }

    void set_velocity()
    {
        // SIM VERSION: just log instead of touching GPIO/PWM.
        ROS_INFO("Obstacle nav velocities -> R:%d  L:%d", m_rightVelocity, m_leftVelocity);

        // REAL ROBOT VERSION:
        // digitalWrite(m_fwRight->m_dirPin, HIGH);
        // digitalWrite(m_fwLeft->m_dirPin, HIGH);
        // digitalWrite(m_bwRight->m_dirPin, HIGH);
        // digitalWrite(m_bwLeft->m_dirPin, HIGH);

        // softPwmWrite(m_fwRight->m_pwmPin, abs(m_rightVelocity));
        // softPwmWrite(m_fwLeft->m_pwmPin, abs(m_leftVelocity));
        // softPwmWrite(m_bwRight->m_pwmPin, abs(m_rightVelocity));
        // softPwmWrite(m_bwLeft->m_pwmPin, abs(m_leftVelocity));
    }

    void stopMotors()
    {
        // For sim: just zero and log.
        m_rightVelocity = 0;
        m_leftVelocity  = 0;
        ROS_INFO("Stopping motors (final) -> R:%d  L:%d", m_rightVelocity, m_leftVelocity);

        // REAL ROBOT VERSION:
        // m_rightVelocity = abs(m_rightVelocity);
        // while(m_rightVelocity >= 0)
        // {
        //     softPwmWrite(m_fwRight->m_pwmPin, m_rightVelocity);
        //     softPwmWrite(m_fwLeft->m_pwmPin,  m_rightVelocity);
        //     softPwmWrite(m_bwRight->m_pwmPin, m_rightVelocity);
        //     softPwmWrite(m_bwLeft->m_pwmPin,  m_rightVelocity);
        //     m_rightVelocity--;
        // }
        // digitalWrite(m_fwRight->m_pwmPin, LOW);
        // digitalWrite(m_fwLeft->m_pwmPin, LOW);
        // digitalWrite(m_bwRight->m_pwmPin, LOW);
        // digitalWrite(m_bwLeft->m_pwmPin, LOW);
        // delay(3);
    }    
};

std::vector<float> laserScanData;

std::vector<float> slicing(std::vector<float>& arr, int X, int Y)
{
    // Take subvector arr[X..Y] inclusive
    auto start = arr.begin() + X;
    auto end   = arr.begin() + Y + 1;

    std::vector<float> result(Y - X + 1);
    copy(start, end, result.begin());
    return result;
}

void laser_scan_callback(const sensor_msgs::LaserScan::ConstPtr &data)
{
    laserScanData = data->ranges;

    std::vector<float> left;
    std::vector<float> right;
    std::vector<float> front;

    // the scan is split into 3 regions: L / F / R
    left  = slicing(laserScanData,   0, 159);
    front = slicing(laserScanData, 160, 319);
    right = slicing(laserScanData, 320, 479);

    bool left_obs_flag  = 0;
    bool front_obs_flag = 0;
    bool right_obs_flag = 0;

    int left_count  = 0;
    int front_count = 0;
    int right_count = 0;

    int thresh1 = 2;  // distance threshold (meters)
    for (int i = 0; i < 160; i++)
        if (left[i] != 0 && left[i] < thresh1)
            left_count++;

    for (int j = 0; j < 160; j++)
        if (front[j] != 0 && front[j] < thresh1)
            front_count++;

    for (int k = 0; k < 160; k++)
        if (right[k] != 0 && right[k] < thresh1)
            right_count++;

    int pixel_thresh{10};

    if (left_count  > pixel_thresh)  left_obs_flag  = 1;
    if (front_count > pixel_thresh)  front_obs_flag = 1;
    if (right_count > pixel_thresh)  right_obs_flag = 1;

    // Obstacle avoidance decision logic
    if (front_obs_flag == 1)
    {
        if (right_obs_flag == 1 && left_obs_flag == 0)
        { 
            std::cout << "Turn Left" << std::endl;
            flag = 2;
        }
        else if (right_obs_flag == 0)
        {
            std::cout << "Turn Right" << std::endl; // default
            flag = 1;
        }
        else
        {
            std::cout << "U Turn" << std::endl;
            flag = 3;
        }
    }
    else
    {
        std::cout << "Go Forward" << std::endl;
        flag = 0; 
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "main_obs");  // <<< small rename so it doesn't collide with main.cpp's "main"
    ros::NodeHandle n;

    ros::Rate loop_rate(10000); // Hz

    ros::Subscriber laserScanSub{};
    int queue_size = 10000;
    laserScanSub = n.subscribe("scan", queue_size, laser_scan_callback);

    delay(2000);  // this now calls our stub above, so it compiles on Ubuntu

    // REAL ROBOT:
    // if(wiringPiSetup()<0)
    // {
    //     std::cout<<"setup wiring pi failed"<<'\n';
    //     return 1;
    // }

    motorControl fwRight{5, 4};   // dir, pwm
    motorControl fwLeft{3, 2};
    motorControl bwRight{29, 28};
    motorControl bwLeft{25, 24};

    Navigation navigation{&n, &fwRight, &fwLeft, &bwRight, &bwLeft};

    int count{0};
    std::cout << "Starting obstacle avoidance loop...\n";   

    while (ros::ok() && count < 500000)
    {
        navigation.navigate_to_point();

        ros::spinOnce();
        loop_rate.sleep();
        count++;
    }

    navigation.stopMotors();
    std::cout << "Stopping motors\n";

    return 0;
}
