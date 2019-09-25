#include "trajectory_generator/trajectory_generator.h"

TrajectoryGenerator::TrajectoryGenerator()
     : path_sub_(nh_, "/path_planning/path", 1)
     , vertice_sub_(nh_, "/vertice/data", 1)
     , sync_(Sync(10), path_sub_, vertice_sub_){

    sync_.registerCallback(boost::bind(&TrajectoryGenerator::inputUtilsCb, this, _1, _2));
//    path_.header.seq = 0;
}

TrajectoryGenerator::~TrajectoryGenerator(){

}

void TrajectoryGenerator::inputUtilsCb(const msgs::PathConstPtr &_path, const msgs::VerticeDataConstPtr &_vertice){
    path_ = *_path;
    vertice_ = *_vertice;

    std::cout << path_.header.seq << " ; " << vertice_.header.seq<< std::endl;
}

void TrajectoryGenerator::getKnots(){
    Point curr_knot{path_.path.back().x, path_.path.back().y};
    Point pres_pt;
    knots_.push_back(curr_knot);
    double y(0);
    double x(0);
    int knot_idx(path_.path.size()-1);
//    std::cout << "Size : " << path_.path.size() << std::endl;
    for(int idx(path_.path.size()-2); idx >= 0; idx--){
        pres_pt = Point{path_.path[idx].x, path_.path[idx].y};
        double dx(pres_pt.first - curr_knot.first);
        double grad((pres_pt.second - curr_knot.second) / (dx == 0 ? dx + 1e-6  : dx));
        std::cout << "Current Knot : " << curr_knot.first << " ; " << curr_knot.second << std::endl;
        std::cout << pres_pt.first << " , " << pres_pt.second << " ; " << grad << std::endl;
        for(int idx2(knot_idx); idx2 >= idx; idx2--){
            if(std::fabs(grad) <= .5){
                x = path_.path[idx2].x;
                y = (double)curr_knot.second + grad * (x - (double)curr_knot.first);
            }else{
                y = path_.path[idx2].y;
                x = (double)curr_knot.first + (y - (double)curr_knot.second) / grad;
            }
            std::cout << "IDX : " << idx << " ; " << idx2 << std::endl;
            std::cout << "(X,Y)" << x << "," << y << std::endl;
            if(vertice_.data[flatIdx(Point{(int)x, (int)y})].state == 2){// Occupied
                std::cout << "There is obstacle !!!" << std::endl;
                curr_knot = Point{pres_pt.first, pres_pt.second};
                knot_idx = idx;
                knots_.push_back(curr_knot);
                break;
            }
        }
    }
    knots_.emplace_back(Point{path_.path.front().x, path_.path.front().y});
}

void TrajectoryGenerator::process(){
    static auto seq(0);
    if(path_.header.seq <= seq)return;

    getKnots();

    for(auto k:knots_)
        std::cout << "Point : " << k.first << " ; " << k.second << std::endl;

    knots_.clear();

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
