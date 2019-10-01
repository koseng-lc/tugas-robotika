#include "odometry/odometry.h"

int main(int argc, char** argv){
    ros::init(argc, argv, "odometry_node");

    Odometry odometry;
    odometry.routine();

    return 0;
}
