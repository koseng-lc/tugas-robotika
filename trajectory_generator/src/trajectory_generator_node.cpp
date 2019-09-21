#include "trajectory_generator/trajectory_generator.h"

int main(int argc, char** argv){

    ros::init(argc, argv, "trajectory_generator_node");

    ros::Rate loop_rate(30);
    while(ros::ok()){
        loop_rate.sleep();
    }

    return 0;
}
