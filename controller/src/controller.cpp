#include "controller/controller.h"

Controller::Controller(){

}

Controller::~Controller(){
    nh_.shutdown();
    ros::shutdown();

}

void Controller::start(){
//    routine_thread_ = boost::thread{boost::bind(&Controller::routine, this)};
}

void Controller::motorVelCb(const msgs::MotorVelConstPtr &_msg){
//    boost::mutex::scoped_lock lk(motor_vel_mtx_);
    motor_vel_data_ = *_msg;
    motor_vel_pub_.publish(motor_vel_data_);
    std::cout << "MVEL : " << motor_vel_data_.motor1 << " ; " << motor_vel_data_.motor2 << " ; " << motor_vel_data_.motor3 << std::endl;
}

void Controller::routine(){

    nh_.setCallbackQueue(&custom_cb_);

    motor_vel_sub_ = nh_.subscribe("/motor/vel", 1, &Controller::motorVelCb, this);

    motor_vel_pub_ = g_nh_.advertise<msgs::MotorVel>("/vrep/motor/vel",1);

    ros::AsyncSpinner spinner(1, &custom_cb_);

    spinner.start();
//    std::cout << "TEST1" << std::endl;
    ros::Rate loop_rate(120);

    while(ros::ok()){

        loop_rate.sleep();
    }
//    std::cout << "TEST2" << std::endl;
    ros::waitForShutdown();
//    std::cout << "TEST3" << std::endl;
}
