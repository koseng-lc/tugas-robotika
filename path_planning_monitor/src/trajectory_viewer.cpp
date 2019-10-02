#include "path_planning_monitor/trajectory_viewer.h"

TrajectoryViewer::TrajectoryViewer(QWidget* parent)
    : QDialog(parent)
    , background_(500,500,QImage::Format_RGB888){

    background_.fill(QColor(100,100,100));

    scene_ = new QGraphicsScene;
    scene_->addPixmap(QPixmap::fromImage(background_));

    view_ = new QGraphicsView;
    view_->setScene(scene_);
    view_->setFixedSize(500,500);
    view_->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    view_->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

    main_layout_ = new QGridLayout;
    main_layout_->addWidget(view_,0,0,1,1);
    main_layout_->addItem(new QSpacerItem(0,0,QSizePolicy::Expanding,QSizePolicy::Expanding),1,1);

    this->setLayout(main_layout_);
    this->setFixedSize(523,523);

    connect(this, SIGNAL(updateScene()), this, SLOT(updateSceneAct()));

    //-- ROS

    solution_sub_ = g_nh_.subscribe("/trajectory/solution", 1, &TrajectoryViewer::solutionCb, this);
    robot_pose_sub_ = g_nh_.subscribe("/robot/pose", 1, &TrajectoryViewer::robotPoseCb, this);

    robot_pose_.x = -1;
    robot_pose_.y = -1;

    //--

    //refresh scene
    scene_->update();

}

TrajectoryViewer::~TrajectoryViewer(){

}

void TrajectoryViewer::solutionCb(const msgs::QuadraticSplineConstPtr &_msg){
    solution_ = *_msg;

    emit updateScene();
}

void TrajectoryViewer::robotPoseCb(const geometry_msgs::Pose2DConstPtr &_msg){
    robot_pose_ = *_msg;

    emit updateScene();
}

void TrajectoryViewer::updateSceneAct(){
    scene_->clear();

    double ly;
    double uy;
    double lx;
    double ux;

    for(std::size_t i(0); i < solution_.f.size(); i++){
        if(solution_.lower_boundy[i] < solution_.upper_boundy[i]){
            ly = solution_.lower_boundy[i];
            uy = solution_.upper_boundy[i];
        }else{
            uy = solution_.lower_boundy[i];
            ly = solution_.upper_boundy[i];
        }

        if(solution_.lower_boundx[i] < solution_.upper_boundx[i]){
            lx = solution_.lower_boundx[i];
            ux = solution_.upper_boundx[i];
        }else{
            ux = solution_.lower_boundx[i];
            lx = solution_.upper_boundx[i];
        }

        background_.setPixel(lx,ly,QColor(0,0,0).rgba());
        background_.setPixel(ux,uy,QColor(0,0,0).rgba());

        //        for(auto y(ly); y < uy; y++){
        msgs::Quadratic f(solution_.f[i]);
        for(auto x(lx); x < ux; x++){
            background_.setPixel(x, f.a * x * x + f.b * x + f.c, QColor(255,0,0).rgba());
        }

//        }
    }

    scene_->addPixmap(std::move(QPixmap::fromImage(background_)));

    for(std::size_t i(0); i < solution_.f.size(); i++){
        scene_->addLine(solution_.lower_boundx[i], solution_.lower_boundy[i],
                        solution_.upper_boundx[i], solution_.upper_boundy[i],QPen(Qt::magenta));
    }

    if(robot_pose_.x != - 1 && robot_pose_.y != -1){
        constexpr auto ROBOT_RAD(5);
        scene_->addEllipse(robot_pose_.x - ROBOT_RAD, robot_pose_.y - ROBOT_RAD,
                           ROBOT_RAD << 1, ROBOT_RAD << 1,
                           QPen(Qt::yellow));
    }

    scene_->update();
}
