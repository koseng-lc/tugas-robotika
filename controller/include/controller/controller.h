#pragma once

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <msgs/MotorVel.h>

#include <boost/thread.hpp>

class Controller{
public:
    Controller();
    ~Controller();
    void start();
private:
    ros::NodeHandle g_nh_;
    ros::NodeHandle nh_;
    ros::CallbackQueue custom_cb_;
    ros::Publisher motor_vel_pub_;
    ros::Subscriber motor_vel_sub_;
    void motorVelCb(const msgs::MotorVelConstPtr& _msg);
    msgs::MotorVel motor_vel_data_;

    boost::mutex routine_mtx_;
    boost::thread routine_thread_;
    void routine();

};
