#include "odometry.h"

Odometry::Odometry()
{
    enable_ = false;
    motor_velocity(3,1);

    config_path_ = ros::package::getPath("kinematics") + "/Data/config.yaml";
    loadConfig();

    boost::thread queue_thread_  = boost::thread(boost::bind(&Odometry::queueThread, this));
}

Odometry::~Odometry()
{
}

void Odometry::queueThread()
{
    ros::Subscriber command_sub_ = nh_.subscribe("/odometry/command", 0, &Odometry::commandCallback, this);
    ros::Subscriber motor_velocity_sub_ = nh_.subscribe("/kinematics/motor_velocity", 0, &Odometry::motorVelocityCallback, this);

    odometry_pub_ = nh_.advertise<odometry_msgs::odometry>("/kinematics/odometry", 2);
}

void Odometry::commandCallback(const std_msgs::String::ConstPtr &_msg)
{
    if(_msg->data == "start")
        enable_ = true;
    else if(_msg->data == "stop")
        enable_ = false;
}

void Odometry::motorVelocityCallback(const geometry_msgs::Vector3::ConstPtr &_msg)
{
    motor_velocity << _msg->x,
                      _msg->y,
                      _msg->z;
}

void Odometry::loadConfig()
{
    YAML::Node config_node;

    try
    {
        config_node = YAML::LoadFile(config_path_.c_str());
    }
    catch(const std::exception& e)
    {
        ROS_ERROR("Failed to load kinematic config");
    }

    YAML::Node config_ = config_node["Robot_Data"];

    base_length = config_["Base_Length"].as<double>();
    wheel_radius = config_["Wheel_Radius"].as<double>();
}

void Odometry::process()
{
    if(!enable_)
        return;
}

inline double Odometry::radToDeg(double rad)
{
    return rad * 180 / PI;
}

inline double Odometry::degToRad(double deg)
{
    return deg * PI / 180;
}



