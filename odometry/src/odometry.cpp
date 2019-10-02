#include "odometry/odometry.h"

Odometry::Odometry(){

}

Odometry::~Odometry(){

}

void Odometry::odometryInCb(const msgs::MotorVelConstPtr &_msg){
    static auto t1(boost::chrono::high_resolution_clock::now());

    motor_vel_ = *_msg;

    RobotVel robot_vel(Kinematics::inst().forwardKinematics(
                           MotorVel{
                               motor_vel_.motor1,
                               motor_vel_.motor2,
                               motor_vel_.motor3
                           }));

    auto t2(boost::chrono::high_resolution_clock::now());
    auto elapsed_time(boost::chrono::duration_cast<boost::chrono::milliseconds>(t2-t1).count());
    t1 = t2;
    msgs::Odometry odometry;
    odometry.dx = robot_vel(0) * elapsed_time;
    odometry.dy = robot_vel(1) * elapsed_time;

    odometry_out_pub_.publish(odometry);
}

void Odometry::process(){



}

void Odometry::routine(){

    odometry_in_sub_ = g_nh.subscribe("/odometry/in", 100, &Odometry::odometryInCb, this);

    odometry_out_pub_ = g_nh.advertise<msgs::Odometry >("/odometry/out", 1);

    ros::Rate loop_rate(SPIN_RATE);
    while(ros::ok()){
        ros::spinOnce();
//        process();
        loop_rate.sleep();
    }
}
