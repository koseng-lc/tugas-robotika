/**
*   @author : koseng (Lintang)
*   @brief : Simple implementation of Dijkstra Algorithm for path planning
*/

/*
 * Update connectivity from 8 to 4 which is approximation of L1 metric
*/

#pragma once

#include <ros/ros.h>
#include <msgs/GridMapData.h>
#include <msgs/VerticeData.h>
#include <msgs/PlannerInput.h>

#include <boost/thread.hpp>
#include <boost/graph/adjacency_list.hpp>

#include "path_planning/solver.h"

class Dijkstra:public Solver{
public:
    Dijkstra();
    ~Dijkstra();

private:
    void solve();
    void solvePerStep();
    void reinit();

    Vertex curr_vtx_;
    Vertex target_vtx_;
    Vertex next_;
    std::queue<Vertex > unvisited_set_;

    boost::mutex finished_mtx_;
};

