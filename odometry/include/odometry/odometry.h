/**
*   @author : koseng (Lintang)
*   @brief : X-Y Odometry
*/


#pragma once

#include <ros/ros.h>
#include <msgs/MotorVel.h>
#include <msgs/Odometry.h>

#include <boost/chrono.hpp>

#include "kinematics/kinematics.h"

#define SPIN_RATE 120 // Hz

#define MSEC2SEC 0.001

class Odometry{
public:
    Odometry();
    ~Odometry();

    void process();
    void routine();

private:
    ros::NodeHandle g_nh;

    ros::Subscriber odometry_in_sub_;
    msgs::MotorVel motor_vel_;

    void odometryInCb(const msgs::MotorVelConstPtr &_msg);

    double odometry__[2];

    ros::Publisher odometry_out_pub_;

};
