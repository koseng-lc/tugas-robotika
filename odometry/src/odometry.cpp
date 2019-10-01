#include "odometry/odometry.h"

Odometry::Odometry(){

}

Odometry::~Odometry(){

}

void Odometry::odometryInCb(const msgs::MotorVelConstPtr &_msg){
    motor_vel_ = *_msg;
}

void Odometry::process(){

    RobotVel robot_vel(Kinematics::inst().forwardKinematics(
                           MotorVel{
                               motor_vel_.motor1,
                               motor_vel_.motor2,
                               motor_vel_.motor3
                           }));

    msgs::Odometry odometry;
    odometry.dx = robot_vel(0) * SPIN_RATE;
    odometry.dy = robot_vel(1) * SPIN_RATE;

    odometry_out_pub_.publish(odometry);

}

void Odometry::routine(){

    odometry_in_sub_ = g_nh.subscribe("/odometry/in", 1, &Odometry::odometryInCb, this);

    odometry_out_pub_ = g_nh.advertise<msgs::Odometry >("/odometry/out", 1);

    ros::Rate loop_rate(SPIN_RATE);
    while(ros::ok()){
        ros::spinOnce();
        process();
        loop_rate.sleep();
    }
}
