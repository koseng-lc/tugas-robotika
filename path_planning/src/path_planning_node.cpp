#include "path_planning/path_planning.h"

int main(int argc, char** argv){

    ros::init(argc, argv, "path_planning_node");

    PathPlanning planner;
    planner.routine();

    return 0;
}
