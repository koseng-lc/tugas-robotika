#include "controller/controller.h"

int main(int argc, char** argv){

    ros::init(argc, argv, "controller_node");

    Controller controller;

    controller.start();

    return 0;
}
