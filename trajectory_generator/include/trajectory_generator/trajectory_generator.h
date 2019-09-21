#pragma once

#include <ros/ros.h>
#include <msgs/MotorVel.h>
#include <msgs/Path.h>

class TrajectoryGenerator{
public:
    TrajectoryGenerator();
    ~TrajectoryGenerator();
private:
    ros::NodeHandle nh_;

    msgs::Path path_;
    ros::Subscriber path_sub_;
    void pathCb(const msgs::PathConstPtr &_msg);

    ros::Publisher motor_vel_pub_;
};
