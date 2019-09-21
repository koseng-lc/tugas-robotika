#include <ros/ros.h>
#include <std_msgs/Float32.h>

#include <msgs/MotorVel.h>

#include "test/test.h"

auto sim_time(.0f);

void testCb(const std_msgs::Float32ConstPtr msg){
    sim_time = msg->data;
}

int main(int argc, char** argv){

    ros::init(argc, argv, "test_node");

    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/simulationTime", 1, &testCb);

    ros::Publisher motor_vel_pub = nh.advertise<msgs::MotorVel>("/motor/vel",1);

    ros::Rate loop_rate(30);

    while(ros::ok()){
        ros::spinOnce();

        msgs::MotorVel motor_vel;
        motor_vel.motor1 = 5000;
        motor_vel.motor2 = -5000;
        motor_vel.motor3 = 5000;
        motor_vel_pub.publish(motor_vel);

        std::cout << "Sim Time : " << sim_time << std::endl;

        loop_rate.sleep();
    }

    return 0;
}
