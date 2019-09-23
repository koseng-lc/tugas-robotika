#include "path_planning/path_planning.h"

PathPlanning::PathPlanning()
    : gmd_pub_(nh_.advertise<msgs::GridMapData >("/grid_map/data", 1))
    , vd_pub_(nh_.advertise<msgs::VerticeData >("/vertice/data", 3))
    , gmd_sub_(nh_.subscribe("/grid_map/data", 1, &PathPlanning::mapCb, this))
    , vd_sub_(nh_.subscribe("/vertice/data", 1, &PathPlanning::verticeDataCb, this))
    , planner_in_sub_(nh_.subscribe("/path_planning/input", 1, &PathPlanning::plannerInCb, this))
    , path_pub_(nh_.advertise<msgs::Path >("/path_planning/path", 1)){

    solver_ = new Dijkstra;    

    graph_ = solver_->getGraph();
    ogm_ = solver_->getOGM();

    map_data_.m.resize(CELL_COLS * CELL_ROWS);
    vertice_data_.data.resize(CELL_COLS * CELL_ROWS);

}

PathPlanning::~PathPlanning(){
    delete solver_;
}

void PathPlanning::plannerInCb(const msgs::PlannerInputConstPtr &_in){
    Point source{_in->source.x, _in->source.y};
    Point target{_in->target.x, _in->target.y};
    solver_->setDelay() = _in->delay;
    solver_->setSource() = source;
    solver_->setTarget() = target;

    path_.header = _in->header;

    std::future<void> concurrency = std::async(std::launch::async, [this,target]{
        solver_->reinit();
        solver_->solve();
        extractSolution(target);
        publishSolution();
        publishData();
    });
}

void PathPlanning::mapCb(const msgs::GridMapDataConstPtr &_map_data){
    map_data_ = *_map_data;

    std::future<void> concurrency = std::async(std::launch::async, [this]{
        Vertex v;
        int idx;
        for(int x(0); x < CELL_COLS; x++){
            for(int y(0); y < CELL_ROWS; y++){
                idx = Solver::flatIdx(x, y);
                ogm_->map_(x, y) = map_data_.m[idx];
                if(ogm_->map_(x, y) == 1.0){
                    v = boost::vertex(idx, *graph_);
                    (*graph_)[v].state = Occupied;
                }
            }
        }
    });
}

void PathPlanning::verticeDataCb(const msgs::VerticeDataConstPtr &_vertice_data){
    vertice_data_ = *_vertice_data;
}

void PathPlanning::routine(){
    ros::Rate loop_rate(60);

    while(ros::ok()){
        ros::spinOnce();

        loop_rate.sleep();
    }
}

void PathPlanning::extractSolution(Point _p){
    auto v{boost::vertex(Solver::flatIdx(_p.first, _p.second), *graph_)};

    geometry_msgs::Point p;
    p.x = _p.first;
    p.y = _p.second;
    path_.path.push_back(p);

    if((*graph_)[v].prev_idx != -1){
        (*graph_)[v].state = Solution;

        auto prev{boost::vertex((*graph_)[v].prev_idx, *graph_)};
        extractSolution(Point{(*graph_)[prev].x, (*graph_)[prev].y});
    }
}

void PathPlanning::publishSolution(){
    path_pub_.publish(path_);
    path_.path.clear();
}

void PathPlanning::publishData(){
    int idx;
    Vertex v;
    for(int x(0); x < CELL_COLS; x++){
        for(int y(0); y < CELL_ROWS; y++){
            idx = Solver::flatIdx(x, y);
            v = boost::vertex(idx, *graph_);
            map_data_.m[idx] = ogm_->map_(x, y);
            vertice_data_.data[idx].prev_idx = (*graph_)[v].prev_idx;
            vertice_data_.data[idx].state = (*graph_)[v].state;
            vertice_data_.data[idx].total_dist = (*graph_)[v].total_dist;
        }
    }

    ros::Time now(ros::Time::now());
    map_data_.header.seq++;
    map_data_.header.stamp = now;
    map_data_.header.frame_id = "path_planning";

    vertice_data_.header.seq++;
    vertice_data_.header.stamp = now;
    vertice_data_.header.frame_id = "path_planning";

    gmd_pub_.publish(map_data_);
    vd_pub_.publish(vertice_data_);
}
