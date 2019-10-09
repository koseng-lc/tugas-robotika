#include "path_planning_monitor/ogmview.h"

const int OGMView::VIEW_HEIGHT(500);
const int OGMView::VIEW_WIDTH(500);

OGMView::OGMView(QWidget* parent)
    : QGraphicsView(parent)
    , ogm_scene_(new QGraphicsScene)
    , gmd_pub_(nh_.advertise<msgs::GridMapData >("/grid_map/data", 1))
    , vd_sub_(nh_.subscribe("/vertice/data", 1, &OGMView::verticeDataCb, this))
    , planner_in_pub_(nh_.advertise<msgs::PlannerInput >("/path_planning/input", 1)){

    init();
}

OGMView::~OGMView(){    
//    delete ogm_;
}

void OGMView::init(){
    this->setScene(ogm_scene_);
    this->setFixedSize(VIEW_WIDTH + 3, VIEW_HEIGHT + 3);

    vertice_data_.data.resize(CELL_ROWS * CELL_COLS);
    for(auto& v:vertice_data_.data){
        v.prev_idx = -1;
        v.state = Unoccupied;
        v.total_dist = std::numeric_limits<double>::max();
    }

    map_data_.m.resize(CELL_ROWS * CELL_COLS);
    for(auto& m:map_data_.m)
        m = .0;

    start_ = Point{-1, -1};
    dest_ = Point{-1, -1};

}

void OGMView::verticeDataCb(const msgs::VerticeDataConstPtr& _vd){
    vertice_data_ = *_vd;
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
//            v.state = Source;
        }break;
        case Destination:{
            auto& last_dest(vertice_data_.data[flatIdx(dest_)]);
            if(last_dest.state != Solution)
                last_dest.state = Unoccupied;
            dest_ = map_pos;
//            v.state
        }break;
        case SetOccupancy:{
            map_data_.m[idx] = 1.0;
        }break;
        case DelOccupancy:{
            map_data_.m[idx] = .0;
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
    int idx(0);
    constexpr int X_CENTER(CELL_COLS >> 1);
    constexpr int Y_CENTER(CELL_ROWS >> 1);
    for(int w(0); w < VIEW_WIDTH; w += CELL_SIZE){
        for(int h(0); h < VIEW_HEIGHT; h += CELL_SIZE){
            map_x = w / CELL_SIZE;
            map_y = h / CELL_SIZE;

            if(map_x == X_CENTER && map_y == Y_CENTER){
                ogm_scene_->addRect(w, h, CELL_SIZE, CELL_SIZE, QPen(Qt::darkBlue), QBrush(Qt::magenta));
                continue;
            }            

            idx = flatIdx(Point{map_x, map_y});

            ogm_scene_->addRect(w, h, CELL_SIZE, CELL_SIZE, QPen(Qt::darkBlue),
                                QBrush(QColor(155.0 * map_data_.m[idx],
                                              135.0 * map_data_.m[idx],
                                              12.00 * map_data_.m[idx])));


            if(start_ != Point{-1, -1} && start_ == Point{map_x, map_y})
                ogm_scene_->addRect(w, h, CELL_SIZE, CELL_SIZE, QPen(Qt::darkBlue), QBrush(Qt::yellow));

            if(dest_ != Point{-1, -1} && dest_ == Point{map_x, map_y})
                ogm_scene_->addRect(w, h, CELL_SIZE, CELL_SIZE, QPen(Qt::darkBlue), QBrush(Qt::green));

            auto v(vertice_data_.data[flatIdx(Point{map_x, map_y})]);

            switch(v.state){
//            case Source:{ogm_scene_->addRect(w, h, CELL_SIZE, CELL_SIZE, QPen(Qt::darkBlue), QBrush(Qt::yellow));}break;
//            case Target:{ogm_scene_->addRect(w, h, CELL_SIZE, CELL_SIZE, QPen(Qt::darkBlue), QBrush(Qt::green));}break;
//            case Occupied:{ogm_scene_->addRect(w, h, CELL_SIZE, CELL_SIZE, QPen(Qt::darkBlue), QBrush(Qt::darkYellow));}break;
//            case Unoccupied:{ogm_scene_->addRect(w, h, CELL_SIZE, CELL_SIZE, QPen(Qt::darkBlue), QBrush(Qt::black));}break;
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

//    extractSolution(flatIdx(dest_));
}

void OGMView::solve(){

    if(start_ == Point{-1,-1} || dest_ == Point{-1,-1}){
        QMessageBox mb;
        mb.setWindowTitle("Error");
        mb.setText("Unable to solve path planning");
        mb.setInformativeText("Please select the start or destination point !");
        mb.setStandardButtons(QMessageBox::Ok);
        mb.setDefaultButton(QMessageBox::Ok);
        int ret(mb.exec());
        (void)ret;
        return;
    }

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
