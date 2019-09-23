#include "trajectory_generator/trajectory_generator.h"

int main(int argc, char** argv){

    ros::init(argc, argv, "trajectory_generator_node");

    TrajectoryGenerator generator;
    generator.routine();

    return 0;
}
