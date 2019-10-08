/**
*   @author : Gabrielle
*   @brief : Three-omniwheel kinematics
*/


#ifndef KINEMATICS_H_
#define KINEMATICS_H_

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
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Vector3.h>

#define PI 3.14159265
#define SPIN_RATE 50

using namespace std;
using namespace Eigen;

class Kinematics
{
public:
    Kinematics();
    ~Kinematics();

    void start();
    void stop();
    void process();

    void queueThread();
    void loadConfig();
    void commandCallback(const std_msgs::String::ConstPtr &_msg);
    void velocityCallback(const geometry_msgs::Pose2D::ConstPtr &_msg);

private:
    ros::NodeHandle nh_;
    boost::thread queue_thread_;
    std::string config_path_;

    ros::Subscriber command_sub_;
    ros::Subscriber velocity_sub_;
    ros::Publisher motor_velocity_pub_;
    inline void pubMotorVelocity(const Eigen::MatrixXd motor_velo);

    bool enable_;
    Eigen::MatrixXd motor;
    Eigen::MatrixXd velocity;
    Eigen::MatrixXd motor_velocity;

    inline double radToDeg(double rad);
    inline double degToRad(double deg);

    double base_length;
    double wheel_radius;
    double velocity_phi;
};

#endif
