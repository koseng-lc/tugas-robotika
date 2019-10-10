/**
*   @author : koseng (Lintang)
*   @brief : Path Planning input-output handle class
*/

#pragma once

#include <ros/ros.h>
#include <msgs/GridMapData.h>
#include <msgs/VerticeData.h>
#include <msgs/PlannerInput.h>
#include <msgs/Path.h>

#include <future>

#include "path_planning/dijkstra.h"

#define PATH_PLANNING_RATE 5

class PathPlanning{
public:
    PathPlanning();
    ~PathPlanning();

    void routine();

private:
    //--------------------------------------------------------------
    ros::NodeHandle nh_;

    ros::Subscriber planner_in_sub_;
    void plannerInCb(const msgs::PlannerInputConstPtr& _in);

    ros::Publisher vd_pub_;

    ros::Subscriber gmd_sub_;
    void mapCb(const msgs::GridMapDataConstPtr &_map_data);

    msgs::GridMapData map_data_;
    msgs::VerticeData vertice_data_;
    void publishData();
    //--------------------------------------------------------------

    void extractSolution(Point _p);

    void publishSolution();
    ros::Publisher path_pub_;
    msgs::Path path_;

    Solver* solver_;
    Graph* graph_;
    OGM* ogm_;

    //--------------------------------------------------------------


};
