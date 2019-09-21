#include "path_planning_monitor/mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow){

    ui->setupUi(this);
    this->removeToolBar(ui->mainToolBar);

    QFile qss_file("/home/koseng/p/test-mp/dark_style_lintang.qss");
    qss_file.open(QFile::ReadOnly);
    QString qss(qss_file.readAll());
    this->setStyleSheet(qss);
    this->setWindowTitle(tr("Path Planner"));

    setupWidgets();

    this->setFocus();

    spin_thread_ = boost::thread{boost::bind(&MainWindow::spinThread, this)};

}

MainWindow::~MainWindow(){
    ros::shutdown();

    boost::mutex::scoped_lock lk(spin_mtx_);
    spin_cv_.wait(lk);
    spin_thread_.join();

    delete ogm_view_;
    delete ui;
}

void MainWindow::spinThread(){
    ros::NodeHandle g_nh;

    motor_vel_pub_ = g_nh.advertise<msgs::MotorVel >("/vrep/motor/vel",1);

    ros::Rate loop_rate(60);

    while(ros::ok()){
        ros::spinOnce();

        loop_rate.sleep();
    }
    spin_cv_.notify_one();
}

void MainWindow::keyPressEvent(QKeyEvent *e){
    int k =  e->key();

    auto vx(.0);
    auto vy(.0);
    auto vz(.0);

    switch(k){
    case Qt::Key_Up:{
        vy = 500;
    }break;
    case Qt::Key_Down:{
        vy = -500;
    }break;
    case Qt::Key_Right:{
        vx = 500;
    }break;
    case Qt::Key_Left:{
        vx = -500;
    }break;
    case Qt::Key_Z:{
        vz = 500;
    }break;
    case Qt::Key_C:{
        vz = -500;
    }break;
    default:break;
    }

    MotorVel motor_vel = Kinematics::inst().inverseKinematics(RobotVel{vx,vy,vz});
    msgs::MotorVel motor_vel_data;
    motor_vel_data.motor1 = motor_vel.at(0);
    motor_vel_data.motor2 = motor_vel.at(1);
    motor_vel_data.motor3 = motor_vel.at(2);

    motor_vel_pub_.publish(motor_vel_data);

}

void MainWindow::updateScene(){    
    ogm_view_->updateScene();
}

void MainWindow::setupWidgets(){
    ogm_view_ = new OGMView;
    ogm_view_->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    ogm_view_->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

    start_rb_ = new QRadioButton;
    start_rb_->setText(tr("Start"));    

    dest_rb_ = new QRadioButton;
    dest_rb_->setText(tr("Destination"));

    set_occupancy_rb_ = new QRadioButton;
    set_occupancy_rb_->setText(tr("Set Occupancy"));

    del_occupancy_rb_ = new QRadioButton;
    del_occupancy_rb_->setText(tr("Del Occupancy"));

    mode_layout_ = new QGridLayout;
    mode_layout_->addWidget(start_rb_,0,0,1,1);
    mode_layout_->addWidget(dest_rb_,1,0,1,1);
    mode_layout_->addWidget(set_occupancy_rb_,2,0,1,1);
    mode_layout_->addWidget(del_occupancy_rb_,3,0,1,1);
    mode_layout_->addItem(new QSpacerItem(0,0,QSizePolicy::Expanding,QSizePolicy::Expanding),4,0);
//    mode_layout_->setRowStretch(4,1);

    mode_gb_ = new QGroupBox;
    mode_gb_->setLayout(mode_layout_);
    mode_gb_->setTitle(tr("Mode"));

    solve_pb_ = new QPushButton;
    solve_pb_->setText(tr("Solve"));

    delay_label_ = new QLabel;
    delay_label_->setText(tr("Delay : "));

    delay_sb_ = new QSpinBox;

    misc_layout_ = new QGridLayout;
    misc_layout_->addWidget(delay_label_,0,0,1,1);
    misc_layout_->addWidget(delay_sb_,   0,1,1,1);
    misc_layout_->addItem(new QSpacerItem(0,0,QSizePolicy::Ignored,QSizePolicy::Expanding),1,2);

    misc_gb_ = new QGroupBox;
    misc_gb_->setLayout(misc_layout_);
    misc_gb_->setTitle(tr("Misc."));

    main_layout_ = new QGridLayout;
    main_layout_->addWidget(ogm_view_,  0,0,4,1);
    main_layout_->addWidget(mode_gb_,   0,1,1,1);
    main_layout_->addWidget(misc_gb_,   1,1,1,1);
    main_layout_->addWidget(solve_pb_,  2,1,1,1);
    main_layout_->addItem(new QSpacerItem(0,0,QSizePolicy::Expanding,QSizePolicy::Expanding),3,2);

    main_widget_ = new QWidget;
    main_widget_->setLayout(main_layout_);

    this->setCentralWidget(main_widget_);

    setupActions();

    //triggering first signal
    delay_sb_->setValue(50);

}

void MainWindow::setupActions(){
    connect(solve_pb_, SIGNAL(clicked(bool)), ogm_view_, SLOT(solve()));

    connect(start_rb_, SIGNAL(clicked(bool)), this, SLOT(modeRBActions()));
    connect(dest_rb_, SIGNAL(clicked(bool)), this, SLOT(modeRBActions()));
    connect(set_occupancy_rb_, SIGNAL(clicked(bool)), this, SLOT(modeRBActions()));
    connect(del_occupancy_rb_, SIGNAL(clicked(bool)), this, SLOT(modeRBActions()));

    connect(delay_sb_, SIGNAL(valueChanged(int)), this, SLOT(setDelay(int)));

    update_timer_.setInterval(33);
    connect(&update_timer_, SIGNAL(timeout()), this, SLOT(updateScene()));
    update_timer_.start();

}

void MainWindow::setDelay(int val){
    ogm_view_->setDelay() = val;
}

void MainWindow::modeRBActions(){
    QObject* subject = sender();
    if(subject == start_rb_){
        ogm_view_->setMode(Start);
    }else if(subject == dest_rb_){
        ogm_view_->setMode(Destination);
    }else if(subject == set_occupancy_rb_){
        ogm_view_->setMode(SetOccupancy);
    }else if(subject == del_occupancy_rb_){
        ogm_view_->setMode(DelOccupancy);
    }
}
