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

    motor_vel_data_.motor1 = std::max(-MAX_MOTOR_SPEED, std::min(motor_vel_data_.motor1, MAX_MOTOR_SPEED));

    motor_vel_data_.motor2 = std::max(-MAX_MOTOR_SPEED, std::min(motor_vel_data_.motor2, MAX_MOTOR_SPEED));

    motor_vel_data_.motor3 = std::max(-MAX_MOTOR_SPEED, std::min(motor_vel_data_.motor3, MAX_MOTOR_SPEED));

    motor_vel_pub_.publish(motor_vel_data_);

}

void Controller::routine(){

    nh_.setCallbackQueue(&custom_cb_);

    motor_vel_sub_ = nh_.subscribe("/motor/vel", 1, &Controller::motorVelCb, this);

    motor_vel_pub_ = g_nh_.advertise<msgs::MotorVel>("/vrep/motor/vel",1);

    ros::AsyncSpinner spinner(1, &custom_cb_);

    spinner.start();

//    ros::Rate loop_rate(CONTROLLER_RATE);

//    while(ros::ok()){

//        loop_rate.sleep();
//    }

    ros::waitForShutdown();

}
