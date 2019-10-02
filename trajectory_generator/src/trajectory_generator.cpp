#include "trajectory_generator/trajectory_generator.h"

TrajectoryGenerator::TrajectoryGenerator()
     : path_sub_(nh_, "/path_planning/path", 1)
     , vertice_sub_(nh_, "/vertice/data", 1)
     , sync_(Sync(10), path_sub_, vertice_sub_)
     , motor_vel_pub_(nh_.advertise<msgs::MotorVel>("/motor/vel", 1))
     , trajectory_pub_(nh_.advertise<msgs::QuadraticSpline>("/trajectory/solution", 1))
     , odometry_sub_(nh_.subscribe("/odometry/out", 1, &TrajectoryGenerator::odometryCb, this))
     , solution(spline_.getSolution())
     , robot_pose_pub_(nh_.advertise<geometry_msgs::Pose2D>("/robot/pose", 1)){

    sync_.registerCallback(boost::bind(&TrajectoryGenerator::inputUtilsCb, this, _1, _2));
//    path_.header.seq = 0;

    //initialize PD Controller
    kP = 0.1;
    kD = 0.01;
    errorx = errory = 0;
    prev_errorx = prev_errory = 0;

    Spline::setX(robot_pos_) = 500 * .5;
    Spline::setY(robot_pos_) = 500 * .5;

}

TrajectoryGenerator::~TrajectoryGenerator(){

}

void TrajectoryGenerator::inputUtilsCb(const msgs::PathConstPtr &_path, const msgs::VerticeDataConstPtr &_vertice){

    path_ = *_path;
    vertice_ = *_vertice;

//    std::cout << path_.header.seq << " ; " << vertice_.header.seq << std::endl;
}

void TrajectoryGenerator::odometryCb(const msgs::OdometryConstPtr &_msg){
    odometry_ = *_msg;
//    std::cout << "Odometer Read : " << odometry_.dx << " ; " << odometry_.dy << std::endl;
    Spline::setX(robot_pos_) += odometry_.dx;
    Spline::setY(robot_pos_) += odometry_.dy;
}

void TrajectoryGenerator::getKnots(){
    Point curr_knot{path_.path.back().x, path_.path.back().y};
    Point pres_pt;
    knots_.push_back(curr_knot);
    double y(0);
    double x(0);
    int knot_idx(path_.path.size() - 1);
//    std::cout << "Size : " << path_.path.size() << std::endl;
    for(int idx(path_.path.size() - 2); idx >= 0; idx--){
        pres_pt = Point{path_.path[idx].x, path_.path[idx].y};
        double dx(pres_pt.first - curr_knot.first);
        double grad((pres_pt.second - curr_knot.second) / (dx == 0 ? 1e-6  : dx));
//        std::cout << "Current Knot : " << curr_knot.first << " ; " << curr_knot.second << std::endl;
//        std::cout << pres_pt.first << " , " << pres_pt.second << " ; " << grad << std::endl;
        for(int idx2(knot_idx); idx2 >= idx; idx2--){
            if(std::fabs(grad) <= .5){
                x = path_.path[idx2].x;
                y = (double)curr_knot.second + grad * (x - (double)Spline::getX(curr_knot));
            }else{
                y = path_.path[idx2].y;
                x = (double)curr_knot.first + (y - (double)curr_knot.second) / grad;
            }
//            std::cout << "IDX : " << idx << " ; " << idx2 << std::endl;
//            std::cout << "(X,Y)" << x << "," << y << std::endl;
            if(vertice_.data[flatIdx(Point{(int)x / CELL_SIZE, (int)y / CELL_SIZE})].state == 2){// Occupied
//                std::cout << "There is obstacle !!!" << std::endl;
                curr_knot = Point{Spline::getX(pres_pt), Spline::getY(pres_pt)};
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
    static std::size_t knots_size(0);

    if(path_.header.seq > seq){
        Spline::setX(robot_pos_) = path_.path.back().x;
        Spline::setY(robot_pos_) = path_.path.back().y;
        getKnots();

        for(auto k:knots_){
            spline_.addPoint(k);
//            std::cout << "Point : " << k.first << " ; " << k.second << std::endl;
        }

        //DEBUG Only
//        spline_.addPoint(Point{3.0,2.5});
//        spline_.addPoint(Point{4.5,1.0});
//        spline_.addPoint(Point{7.0,2.5});
//        spline_.addPoint(Point{9.0,0.5});

        spline_.solve();

        knots_size = knots_.size();

        spline_.clearPoints();
        knots_.clear();

        seq = path_.header.seq;

        trajectory_pub_.publish(*solution);
    }

    if(solution->f.size() == 0) return;

    int target2robot_x(Spline::getX(robot_pos_) - path_.path.front().x);
    int target2robot_y(Spline::getY(robot_pos_) - path_.path.front().y);

    if(std::sqrt(target2robot_x*target2robot_x + target2robot_y*target2robot_y) < DISTANCE_TOLERANCE){
        solution->f.clear();
    }

    //-- Controller

//    int piece_wise_idx(0);
    int min_dist(std::numeric_limits<int>::max());

    double xref;
    double yref;

    //-- first concept
    /*for(std::size_t i(0); i < (knots_size - 1); i++){
        double dx1(Spline::getX(robot_pos_) - Spline::getX(knots_[i]));
        double dy1(Spline::getY(robot_pos_) - Spline::getY(knots_[i]));
        double dx2(Spline::getX(robot_pos_) - Spline::getX(knots_[i+1]));
        double dy2(Spline::getY(robot_pos_) - Spline::getY(knots_[i+1]));

        msgs::Quadratic f(solution->f[i]);

        std::cout << "Dist to piece-wise " << i << " : " << xref << " , " << yref << std::endl;
        if(sgn(dy1/(dx1 == 0 ? 1e-6 : dx1)) != sgn(dy2/(dx2 == 0 ? 1e-6 : dx2))){
            auto dist(std::sqrt(dx1*dx1 + dy1*dy1) + std::sqrt(dx2*dx2 + dy2*dy2));
            if(dist < min_dist){
                piece_wise_idx = i;
                min_dist = dist;
            }
        }

    }

    // Add little bit code here
    // to handle the selected(piece_wise_idx) function
    */

    //-- second concept
    for(std::size_t i(0); i < (knots_size - 1); i++){

        msgs::Quadratic f(solution->f[i]);

        double x = (i == 0 ?
                        (Spline::getY(robot_pos_) - f.c) / f.b :
                        (-f.b + std::sqrt(f.b*f.b - 4.0 * f.a * (f.c - Spline::getY(robot_pos_)))) / (2.0 * f.a));
        double y = (f.a * Spline::getX(robot_pos_) * Spline::getX(robot_pos_) + f.b * Spline::getX(robot_pos_) + f.c);

        std::cout << "Ref to piece-wise " << i << " : " << xref << " , " << yref << std::endl;
        double dx(Spline::getX(robot_pos_) - x);
        double dy(Spline::getY(robot_pos_) - y);
        auto dist(std::sqrt(dx*dx + dy*dy));
        if(dist < min_dist){
            xref = x;
            yref = y;
            min_dist = dist;
        }

    }

    errorx = Spline::getX(robot_pos_) - xref;
    errory = Spline::getY(robot_pos_) - yref;

    prev_errorx = errorx;
    prev_errory = errory;

    auto vdx(path_.path.back().x - path_.path.front().x);
    auto vdy(path_.path.back().y - path_.path.front().y);
//    auto gradient(dy/(dx == 0 ? 1e-6 : dx));
    auto bearing(std::atan2(vdy,vdx));
    auto velx(SPEED * std::cos(bearing));
    auto vely(SPEED * std::sin(bearing));    

    auto inputx(velx + kP * errorx + kD * (errorx - prev_errorx));
    auto inputy(vely + kP * errory + kD * (errory - prev_errory));

    RobotVel rvel;
    rvel << -inputx << endr << inputy << endr << .0 << endr;
    MotorVel mvel(Kinematics::inst().inverseKinematics(rvel));

    motor_vel_.motor1 = mvel(0);
    motor_vel_.motor2 = mvel(1);
    motor_vel_.motor3 = mvel(2);

    std::cout << "===========================================================" << std::endl;
//    std::cout << "Piece-wise idx : " << piece_wise_idx << std::endl;
    for(auto f:solution->f)
        std::cout << f.a << " ; " << f.b << " ; " << f.c << std::endl;
    std::cout << "Robot Pos : " << Spline::getX(robot_pos_) << " ; " << Spline::getY(robot_pos_) << std::endl;
    std::cout << "XRef : " << xref << " ; YRef : " << yref << std::endl;
    std::cout << "Error X : " << errorx << " ; Error Y : " << errory << std::endl;
    rvel.print("RVel : ");
    mvel.print("MVel : ");

    geometry_msgs::Pose2D robot_pose;
    robot_pose.x = Spline::getX(robot_pos_);
    robot_pose.y = Spline::getY(robot_pos_) ;
    robot_pose_pub_.publish(robot_pose);

    motor_vel_pub_.publish(motor_vel_);

    //-- End of Controller

}

void TrajectoryGenerator::routine(){
    ros::Rate loop_rate(30);
    while(ros::ok()){
        ros::spinOnce();
        process();
        loop_rate.sleep();
    }
}
