#include "trajectory_generator/trajectory_generator.h"

TrajectoryGenerator::TrajectoryGenerator()
     : path_sub_(nh_.subscribe("/path_planning/path", 1, &TrajectoryGenerator::pathCb, this)){

}

TrajectoryGenerator::~TrajectoryGenerator(){

}

void TrajectoryGenerator::pathCb(const msgs::PathConstPtr &_msg){
    path_ = *_msg;
}
