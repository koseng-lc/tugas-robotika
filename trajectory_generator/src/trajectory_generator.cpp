#include "trajectory_generator/trajectory_generator.h"

TrajectoryGenerator::TrajectoryGenerator()
     : path_sub_(nh_, "/path_planning/path", 1)
     , vertice_sub_(nh_, "/vertice/data", 1)
     , sync_(Sync(10), path_sub_, vertice_sub_){

    sync_.registerCallback(boost::bind(&TrajectoryGenerator::inputUtilsCb, this, _1, _2));

}

TrajectoryGenerator::~TrajectoryGenerator(){

}

void TrajectoryGenerator::inputUtilsCb(const msgs::PathConstPtr &_path, const msgs::VerticeDataConstPtr &_vertice){
    path_ = *_path;
    vertice_ = *_vertice;
}

void TrajectoryGenerator::getKnots(){
    Point curr_knot{path_.path.front().x, path_.path.front().y};
    Point pres_pt;
    knots_.push_back(curr_knot);
    int y(0);
    int x(0);
    int knot_idx(0);
    for(size_t idx(1); idx < path_.path.size(); idx++){
        pres_pt = Point{path_.path[idx].x, path_.path[idx].y};
        double grad((pres_pt.second - curr_knot.second) / ((pres_pt.first - curr_knot.first) + 1e-6));
        for(size_t idx2(knot_idx); idx2 < idx; idx2++){
            x = path_.path[idx2].x;
            y = pres_pt.second + grad * (x - knot_idx);
            if(vertice_.data[flatIdx(Point{x, y})].state == 2){//Occupied
                curr_knot = Point{x, y};
                knots_.push_back(curr_knot);
            }
        }
    }
    knots_.emplace_back(Point{path_.path.back().x, path_.path.back().y});
}

void TrajectoryGenerator::process(){
    static auto seq(path_.header.seq);
    if(path_.header.seq <= seq)return;



    seq = path_.header.seq;
}

void TrajectoryGenerator::routine(){
    ros::Rate loop_rate(30);
    while(ros::ok()){
        ros::spinOnce();
        process();
        loop_rate.sleep();
    }
}
