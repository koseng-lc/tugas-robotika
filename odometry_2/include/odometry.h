/**
*   @author : Gabrielle
*   @brief : Odometry
*/


#ifndef ODOMETRY_H_
#define ODOMETRY_H_

#include <iostream>
#include <cmath>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/package.h>
#include <boost/thread.hpp>
#include <yaml-cpp/yaml.h>

#include <std_msgs/String.h>
#include <odometry_msgs/odometry.h>
#include <geometry_msgs/Vector3.h>

#define PI 3.14159265
#define SPIN_RATE 50

using namespace std;
using namespace Eigen;

class Odometry
{
public:
    Odometry();
    ~Odometry();

    void start();
    void stop();
    void process();

    void queueThread();
    void loadConfig();
    void commandCallback(const std_msgs::String::ConstPtr &_msg);
    void motorVelocityCallback(const geometry_msgs::Vector3::ConstPtr &_msg);

private:
    ros::NodeHandle nh_;
    boost::thread queue_thread_;
    std::string config_path_;

    ros::Subscriber command_sub_;
    ros::Subscriber motor_velocity_sub_;
    ros::Publisher odometry_pub_;

    bool enable_;
    Eigen::MatrixXd motor_velocity;

    inline double radToDeg(double rad);
    inline double degToRad(double deg);

    double base_length;
    double wheel_radius;
};

#endif
