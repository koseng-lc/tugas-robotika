#include "trajectory_generator/trajectory_generator.h"

TrajectoryGenerator::TrajectoryGenerator()
     : path_sub_(nh_, "/path_planning/path", 1)
     , vertice_sub_(nh_, "/vertice/data", 1)
     , sync_(Sync(10), path_sub_, vertice_sub_)
     , motor_vel_pub_(nh_.advertise<msgs::MotorVel>("/motor/vel", 1))
     , trajectoryx_pub_(nh_.advertise<msgs::QuadraticSpline>("/trajectory/solutionx", 1))
     , trajectoryy_pub_(nh_.advertise<msgs::QuadraticSpline>("/trajectory/solutiony", 1))
     , odometry_sub_(nh_.subscribe("/odometry/out", 1, &TrajectoryGenerator::odometryCb, this))
     , solutionx(spline_x_.getSolution())
     , solutiony(spline_y_.getSolution())
     , robot_pose_pub_(nh_.advertise<geometry_msgs::Pose2D>("/robot/pose", 1)){

    sync_.registerCallback(boost::bind(&TrajectoryGenerator::inputUtilsCb, this, _1, _2));

    //initialize PD Controller
    kP = 3.5;
    kD = 1.5;
    errorx = errory = 0;
    prev_errorx = prev_errory = 0;

    mileage_ = 0;
    piece_wise_idx_ = 0;
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

    Spline::setX(robot_pos_) += -1 * odometry_.dx;
    Spline::setY(robot_pos_) += -1 * odometry_.dy;
    mileage_ += std::sqrt(odometry_.dx * odometry_.dx + odometry_.dy * odometry_.dy);
}

void TrajectoryGenerator::getKnots(){
    Point curr_knot{path_.path.back().x, path_.path.back().y};
    Point pres_pt;
    knots_.push_back(curr_knot);
    curr_knot = Point{path_.path[path_.path.size() - 2].x, path_.path[path_.path.size() - 2].y};
    knots_.push_back(curr_knot);
    double y(.0);
    double x(.0);
    int knot_idx(path_.path.size() - 2);

//    std::cout << "Size : " << path_.path.size() << std::endl;

    for(int idx(path_.path.size() - 3); idx >= 0; idx--){
        pres_pt = Point{path_.path[idx].x, path_.path[idx].y};
        double dx(Spline::getX(pres_pt) - Spline::getX(curr_knot));
        double grad((Spline::getY(pres_pt) - Spline::getX(curr_knot)) / (dx == 0 ? 1e-6  : dx));

//        std::cout << "Current Knot : " << curr_knot.first << " ; " << curr_knot.second << std::endl;
//        std::cout << pres_pt.first << " , " << pres_pt.second << " ; " << grad << std::endl;

        for(int idx2(knot_idx); idx2 >= idx; idx2--){
            if(std::fabs(grad) <= .5){
                x = path_.path[idx2].x;
                y = (double)Spline::getY(curr_knot) + grad * (x - (double)Spline::getX(curr_knot));
            }else{
                y = path_.path[idx2].y;
                x = (double)Spline::getX(curr_knot) + (y - (double)Spline::getY(curr_knot)) / grad;
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

Point TrajectoryGenerator::calcReference(){

    msgs::Quadratic fx(solutionx->f[piece_wise_idx_]);
    msgs::Quadratic fy(solutiony->f[piece_wise_idx_]);
    Point ref{
        fx.a * mileage_ * mileage_ + fx.b * mileage_ + fx.c,
        fy.a * mileage_ * mileage_ + fy.b * mileage_ + fy.c
    };

    if(mileage_ >= solutionx->upper_bound[piece_wise_idx_])
        ++piece_wise_idx_;

    if(piece_wise_idx_ >= solutionx->f.size()){
        piece_wise_idx_ = 0;
        mileage_ = 0;
        piece_wise_idx_ = 0;
        ref = Point{-1.0, -1.0};
    }

    return ref;
}

void TrajectoryGenerator::process(){
    static auto seq(0);
    static std::size_t knots_size(0);

    if(path_.header.seq > seq){
        Spline::setX(robot_pos_) = path_.path.back().x;
        Spline::setY(robot_pos_) = path_.path.back().y;
        getKnots();

        Point xs;
        Point ys;
        auto dist(.0);

        xs = Point{dist, Spline::getX(knots_.front())};
        ys = Point{dist, Spline::getY(knots_.front())};

        spline_x_.addPoint(xs);
        spline_y_.addPoint(ys);

        for(std::size_t i(1); i < knots_.size(); i++){
            dist += Spline::distance(knots_[i], knots_[i-1]);
            xs = Point{dist, Spline::getX(knots_[i])};
            ys = Point{dist, Spline::getY(knots_[i])};
            spline_x_.addPoint(xs);
            spline_y_.addPoint(ys);
        }

        //DEBUG Only
//        spline_x_.addPoint(Point{3.0,2.5});
//        spline_x_.addPoint(Point{4.5,1.0});
//        spline_x_.addPoint(Point{7.0,2.5});
//        spline_x_.addPoint(Point{9.0,0.5});

        spline_x_.solve();
        spline_y_.solve();

        knots_size = knots_.size();

        spline_x_.clearPoints();
        spline_y_.clearPoints();
        knots_.clear();

        seq = path_.header.seq;

        trajectoryx_pub_.publish(*solutionx);
        trajectoryy_pub_.publish(*solutiony);
    }

    if(solutionx->f.size() == 0 ||
            solutiony->f.size() == 0) return;

    int target2robot_x(Spline::getX(robot_pos_) - path_.path.front().x);
    int target2robot_y(Spline::getY(robot_pos_) - path_.path.front().y);

//    msgs::QuadraticSpline test;

    Point ref(calcReference());

    if(std::sqrt(target2robot_x*target2robot_x + target2robot_y*target2robot_y) < DISTANCE_TOLERANCE ||
            ref == Point{-1.0, -1.0}){
        solutionx->f.clear();
        solutiony->f.clear();
    }

    //-- Controller

//    int piece_wise_idx(0);
//    auto min_dist(std::numeric_limits<double>::max());

//    double xref;
//    double yref;

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
    /*auto min_diff(std::numeric_limits<double>::max());
    Point target_point;
    for(std::size_t i(0); i < (knots_size - 1); i++){

        msgs::Quadratic f(solution->f[i]);
        double x(Spline::getX(robot_pos_)),y(Spline::getY(robot_pos_));
        double diff(std::numeric_limits<double>::max());

        if(std::fabs(solution->lower_boundx[i] - solution->upper_boundx[i]) >=
                std::fabs(solution->lower_boundy[i] - solution->upper_boundy[i])){
            if(Spline::getX(robot_pos_) > solution->lower_boundx[i] && Spline::getX(robot_pos_) < solution->upper_boundx[i]){

                y = (f.a * Spline::getX(robot_pos_) * Spline::getX(robot_pos_) + f.b * Spline::getX(robot_pos_) + f.c);
                diff = Spline::getY(robot_pos_) - y;
            }
        }else{
            if(Spline::getY(robot_pos_) > solution->lower_boundy[i] && Spline::getY(robot_pos_) < solution->upper_boundy[i]){

                x = (f.a == 0 ?
                         (Spline::getY(robot_pos_) - f.c) / (f.b == 0 ? 1e-6 : f.b) :
                         (-f.b + std::sqrt(f.b*f.b - 4.0 * f.a * (f.c - Spline::getY(robot_pos_)))) / (2.0 * f.a));
                diff = Spline::getX(robot_pos_) - x;
            }
        }

        if(diff < min_diff){
            xref = x;
            yref = y;
            min_diff = diff;
            target_point = Point{solution->upper_boundx[i], solution->upper_boundy[i]};
        }

    }*/

    //-- third concept
    /*msgs::Quadratic f(solution->f[pi2ece_wise_idx]);

    double x(Spline::getX(robot_pos_)),y(Spline::getY(robot_pos_));
    std::cout << "Lower : " << solution->lower_boundx[piece_wise_idx] << " , " << solution->lower_boundy[piece_wise_idx] << std::endl;
    std::cout << "Upper : " << solution->upper_boundx[piece_wise_idx] << " , " << solution->upper_boundy[piece_wise_idx] << std::endl;

//    double lx()

    if(Spline::getX(robot_pos_) >= solution->lower_boundx[piece_wise_idx] && Spline::getX(robot_pos_) < solution->upper_boundx[piece_wise_idx]){
//        if(std::fabs(solution->lower_boundx[piece_wise_idx] - solution->upper_boundx[piece_wise_idx]) >=
//                std::fabs(solution->lower_boundy[piece_wise_idx] - solution->upper_boundy[piece_wise_idx])){

            y = (f.a * Spline::getX(robot_pos_) * Spline::getX(robot_pos_) + f.b * Spline::getX(robot_pos_) + f.c);
//        }
    }else if(Spline::getY(robot_pos_) >= solution->lower_boundy[piece_wise_idx] && Spline::getY(robot_pos_) < solution->upper_boundy[piece_wise_idx]){
//        if(std::fabs(solution->lower_boundx[piece_wise_idx] - solution->upper_boundx[piece_wise_idx]) <
//                std::fabs(solution->lower_boundy[piece_wise_idx] - solution->upper_boundy[piece_wise_idx])){

            x = (f.a == 0 ?
                     (Spline::getY(robot_pos_) - f.c) / (f.b == 0 ? 1e-6 : f.b) :
                     (-f.b + std::sqrt(f.b*f.b - 4.0 * f.a * (f.c - Spline::getY(robot_pos_)))) / (2.0 * f.a));
//        }
    }else{
        ++piece_wise_idx;
    }*/

    errorx = Spline::getX(robot_pos_) - Spline::getX(ref);
    errory = Spline::getY(robot_pos_) - Spline::getY(ref);

    auto vdx(solutionx->upper_bound[piece_wise_idx_] - Spline::getX(robot_pos_));
    auto vdy(solutiony->upper_bound[piece_wise_idx_] - Spline::getY(robot_pos_));
//    auto gradient(dy/(dx == 0 ? 1e-6 : dx));
    auto bearing(std::atan2(vdy,vdx));
    auto velx(SPEED * std::cos(bearing));
    auto vely(SPEED * std::sin(bearing));    

    auto inputx(velx + kP * errorx + kD * (errorx - prev_errorx));
    auto inputy(vely + kP * errory + kD * (errory - prev_errory));

    prev_errorx = errorx;
    prev_errory = errory;

    RobotVel rvel;
    rvel << inputx << endr << inputy << endr << .0 << endr;
//    MotorVel mvel(Kinematics::inst().inverseKinematics(rvel));
    MotorVel mvel(Kinematics::inst().inverseKinematics_2(rvel));

    motor_vel_.motor1 = mvel(0);
    motor_vel_.motor2 = mvel(1);
    motor_vel_.motor3 = mvel(2);

//    std::cout << "===========================================================" << std::endl;
//    std::cout << "Piece-wise idx : " << piece_wise_idx << std::endl;
//    for(auto f:solution->f)
//        std::cout << f.a << " ; " << f.b << " ; " << f.c << std::endl;
//    for(std::size_t i(0); i < solutionx->f.size(); i++){

//    }
//    std::cout << "Robot Pos : " << Spline::getX(robot_pos_) << " ; " << Spline::getY(robot_pos_) << std::endl;
//    std::cout << "XRef : " << Spline::getX(ref) << " ; YRef : " << Spline::getY(ref) << std::endl;
//    std::cout << "Error X : " << errorx << " ; Error Y : " << errory << std::endl;
//    std::cout << "Input : " << inputx << "," << inputy << std::endl;
//    std::cout << "Mileage : " << mileage_ << std::endl;
//    std::cout << "Bearing : " << (bearing * 180.0 / M_PI) << std::endl;
//    std::cout << "Upper bound : " << solutionx->upper_bound[piece_wise_idx_] << std::endl;

//    rvel.print("RVel : ");
//    mvel.print("MVel : ");

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
