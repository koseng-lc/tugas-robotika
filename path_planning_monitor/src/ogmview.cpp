#include "path_planning_monitor/ogmview.h"

const int OGMView::MAP_HEIGHT(60);
const int OGMView::MAP_WIDTH(90);
const int OGMView::VIEW_HEIGHT(600);
const int OGMView::VIEW_WIDTH(900);

OGMView::OGMView()
    : ogm_scene_(new QGraphicsScene)
    , gmd_pub_(nh_.advertise<msgs::GridMapData >("/grid_map/data", 1))
    , vd_pub_(nh_.advertise<msgs::VerticeData >("/vertice/data", 1))
    , gmd_sub_(nh_, "/grid_map/data", 1)
    , vd_sub_(nh_, "/vertice/data", 1)
    , sync_(SyncPolicy(10), gmd_sub_, vd_sub_)
    , planner_in_pub_(nh_.advertise<msgs::PlannerInput >("/path_planning/input", 1)){

    init();
}

OGMView::~OGMView(){    
//    delete ogm_;
}

void OGMView::init(){
    this->setScene(ogm_scene_);
    this->setFixedSize(VIEW_WIDTH + 3, VIEW_HEIGHT + 3);

    sync_.registerCallback(boost::bind(&OGMView::inputUtilsCb, this, _1, _2));

    vertice_data_.data.resize(MAP_HEIGHT * MAP_WIDTH);
    for(auto& v:vertice_data_.data){
        v.prev_idx = -1;
        v.state = Unoccupied;
        v.total_dist = std::numeric_limits<double>::max();
    }

    map_data_.m.resize(MAP_HEIGHT * MAP_WIDTH);
    for(auto& m:map_data_.m)
        m = .0;

}

void OGMView::inputUtilsCb(const msgs::GridMapDataConstPtr& _gmd, const msgs::VerticeDataConstPtr& _vd){
//    map_data_ = *_gmd;
    vertice_data_ = *_vd;
//    QFuture<void> future = QtConcurrent::run(boost::bind(&OGMView::extract, this));
}

void OGMView::pathCb(const msgs::PathConstPtr &_path){
    path_ = *_path;

}

void OGMView::mousePressEvent(QMouseEvent *e){
    cursor_pos_ = e->pos();
    updateState();
}

void OGMView::mouseMoveEvent(QMouseEvent *e){
    cursor_pos_ = e->pos();
    updateState();
}

void OGMView::updateState(){
    if(cursor_pos_.x() >= 0 &&
        cursor_pos_.y() >= 0 &&
        cursor_pos_.x() < VIEW_WIDTH  &&
        cursor_pos_.y() < VIEW_HEIGHT){

        Point map_pos{cursor_pos_.x() / CELL_SIZE, cursor_pos_.y() / CELL_SIZE};

        int idx(flatIdx(map_pos));

        auto& v(vertice_data_.data[idx]);

        switch(mode_){
        case Start:{
            auto& last_start(vertice_data_.data[flatIdx(start_)]);
            if(last_start.state != Solution)
                last_start.state = Unoccupied;
            start_ = map_pos;
            v.state = Source;            
        }break;
        case Destination:{
            auto& last_dest(vertice_data_.data[flatIdx(dest_)]);
            if(last_dest.state != Solution)
                last_dest.state = Unoccupied;
            dest_ = map_pos;
            v.state = Target;
        }break;
        case SetOccupancy:{
            map_data_.m[idx] = 1.0;
            v.state = Occupied;
        }break;
        case DelOccupancy:{
            map_data_.m[idx] = .0;
            v.state = Unoccupied;
        }break;
        default:break;
        }
    }
    publishMap();
}

void OGMView::updateScene(){
    ogm_scene_->clear();
    int map_x(0);
    int map_y(0);
    for(int w(0); w < VIEW_WIDTH; w += CELL_SIZE){
        for(int h(0); h < VIEW_HEIGHT; h += CELL_SIZE){
            map_x = w / CELL_SIZE;
            map_y = h / CELL_SIZE;

            auto v(vertice_data_.data[flatIdx(Point{map_x, map_y})]);

            switch(v.state){
            case Source:{ogm_scene_->addRect(w, h, CELL_SIZE, CELL_SIZE, QPen(Qt::darkBlue), QBrush(Qt::yellow));}break;
            case Target:{ogm_scene_->addRect(w, h, CELL_SIZE, CELL_SIZE, QPen(Qt::darkBlue), QBrush(Qt::green));}break;
            case Occupied:{ogm_scene_->addRect(w, h, CELL_SIZE, CELL_SIZE, QPen(Qt::darkBlue), QBrush(Qt::darkYellow));}break;
            case Unoccupied:{ogm_scene_->addRect(w, h, CELL_SIZE, CELL_SIZE, QPen(Qt::darkBlue), QBrush(Qt::black));}break;
            case Solution:{ogm_scene_->addRect(w, h, CELL_SIZE, CELL_SIZE, QPen(Qt::darkBlue), QBrush(Qt::red));}break;
            case Visited:{ogm_scene_->addRect(w, h, CELL_SIZE, CELL_SIZE, QPen(Qt::darkBlue), QBrush(Qt::darkCyan));}break;
            case Unvisited:{ogm_scene_->addRect(w, h, CELL_SIZE, CELL_SIZE, QPen(Qt::darkBlue), QBrush(Qt::black));}break;
            default:break;
            }
        }
    }
    ogm_scene_->update();    
}

void OGMView::extract(){

    extractSolution(flatIdx(dest_));
}

void OGMView::solve(){
    planner_in_.source.x = start_.first;
    planner_in_.source.y = start_.second;
    planner_in_.source.z = 0;

    planner_in_.target.x = dest_.first;
    planner_in_.target.y = dest_.second;
    planner_in_.target.z = 0;

    planner_in_.delay = delay_;

    planner_in_pub_.publish(planner_in_);
//    QFuture<void> future = QtConcurrent::run(boost::bind(&OGMView::solveActions, this));
}

void OGMView::setMode(Mode mode){
    mode_ = mode;
}

void OGMView::publishMap(){
    gmd_pub_.publish(map_data_);
}
