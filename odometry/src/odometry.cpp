#include "odometry/odometry.h"

Odometry::Odometry(){
    odometry__[0] = 0;
    odometry__[1] = 0;
}

Odometry::~Odometry(){

}

void Odometry::odometryInCb(const msgs::MotorVelConstPtr &_msg){
    static auto t1(boost::chrono::high_resolution_clock::now());

    motor_vel_ = *_msg;

//    RobotVel robot_vel(Kinematics::inst().forwardKinematics(
//                           MotorVel{
//                               motor_vel_.motor1,
//                               motor_vel_.motor2,
//                               motor_vel_.motor3
//                           }));

    RobotVel robot_vel(Kinematics::inst().forwardKinematics_2(
                           MotorVel{
                               motor_vel_.motor1,
                               motor_vel_.motor2,
                               motor_vel_.motor3
                           }));

    auto t2(boost::chrono::high_resolution_clock::now());
    double elapsed_time(boost::chrono::duration_cast<boost::chrono::milliseconds>(t2-t1).count());
    t1 = t2;
    msgs::Odometry odometry;
    odometry.dx = robot_vel(0) * elapsed_time * MSEC2SEC;
    odometry.dy = robot_vel(1) * elapsed_time * MSEC2SEC;

    double mileage = std::sqrt(odometry.dx*odometry.dx+odometry.dy*odometry.dy);

    odometry__[0]+=(mileage*cos(odometry.dx/mileage));
    odometry__[1]+=(mileage*sin(odometry.dy/mileage));

//    std::cout << "-----> Odometry Debug <-----" << std::endl;
//    std::cout << "Elapsed Time : " << elapsed_time << std::endl;
//    std::cout << "Robot VEL : " << robot_vel(0) << "," << robot_vel(1) << std::endl;
//    std::cout << "Odometry read : " << odometry.dx << " , " << odometry.dy << std::endl;
//    std::cout << "Mileage read : " << odometry__[0] << " , " << odometry__[1] << std::endl;
//    std::cout << "Motor VEL : " << motor_vel_.motor1 << "," << motor_vel_.motor2 << "," << motor_vel_.motor3 << std::endl;
//    std::cout << "----------------------------" << std::endl;

    odometry_out_pub_.publish(odometry);
}

void Odometry::process(){



}

void Odometry::routine(){

    odometry_in_sub_ = g_nh.subscribe("/odometry/in", 100, &Odometry::odometryInCb, this);

    odometry_out_pub_ = g_nh.advertise<msgs::Odometry>("/odometry/out", 1);

    ros::Rate loop_rate(SPIN_RATE);
    while(ros::ok()){
        ros::spinOnce();
//        process();
        loop_rate.sleep();
    }
}
