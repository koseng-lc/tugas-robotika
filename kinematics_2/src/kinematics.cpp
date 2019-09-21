#include "kinematics.h"

Kinematics::Kinematics()
{
    enable_ = false;
    velocity(3,1), motor(3,3), motor_velocity(3,1);

    config_path_ = ros::package::getPath("kinematics") + "/Data/config.yaml";
    loadConfig();

    boost::thread queue_thread_  = boost::thread(boost::bind(&Kinematics::queueThread, this));
}

Kinematics::~Kinematics()
{
}

void Kinematics::queueThread()
{
    ros::Subscriber command_sub_ = nh_.subscribe("/kinematics/command", 0, &Kinematics::commandCallback, this);
    ros::Subscriber velocity_sub_ = nh_.subscribe("/kinematics/velocity", 0, &Kinematics::velocityCallback, this);

    motor_velocity_pub_ = nh_.advertise<geometry_msgs::Pose2D>("/kinematics/motor_velocity", 2);
}

void Kinematics::commandCallback(const std_msgs::String::ConstPtr &_msg)
{
    if(_msg->data == "start")
        enable_ = true;
    else if(_msg->data == "stop")
        enable_ = false;
}

void Kinematics::velocityCallback(const geometry_msgs::Pose2D::ConstPtr &_msg)
{
    velocity << _msg->x,
                _msg->y,
                _msg->theta;

    velocity_phi = atan2(_msg->x, _msg->y);
}

void Kinematics::loadConfig()
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

void Kinematics::process()
{
    if(!enable_)
    {
        motor_velocity << 0, 0, 0;
        pubMotorVelocity(motor_velocity);
        return;
    }

    motor << 1/(wheel_radius*sin(degToRad(300)-velocity_phi)),     1/(wheel_radius*cos(degToRad(300)-velocity_phi)),     1,
             1/(wheel_radius*sin(degToRad(60)-velocity_phi)),      1/(wheel_radius*cos(degToRad(60)-velocity_phi)),      1,
             1/(wheel_radius*sin(degToRad(180)-velocity_phi)),     1/(wheel_radius*cos(degToRad(180)-velocity_phi)),     1;   
    
    motor_velocity = motor * velocity / SPIN_RATE;
    pubMotorVelocity(motor_velocity);

    return;
}

inline void Kinematics::pubMotorVelocity(const Eigen::MatrixXd motor_velocity)
{
    geometry_msgs::Vector3 motor_velocity_;

    motor_velocity_.x = motor_velocity(1,1);
    motor_velocity_.y = motor_velocity(2,1);
    motor_velocity_.z = motor_velocity(1,1);

    motor_velocity_pub_.publish(motor_velocity_);
}

inline double Kinematics::radToDeg(double rad)
{
    return rad * 180 / PI;
}

inline double Kinematics::degToRad(double deg)
{
    return deg * PI / 180;
}



