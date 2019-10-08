/**
*   @author : koseng (Lintang)
*   @brief : Trajectory Generator to get robot trajectory reference + PD Controller
*/

#pragma once

#include <ros/ros.h>
#include <msgs/MotorVel.h>
#include <msgs/Odometry.h>
#include <msgs/Path.h>
#include <msgs/VerticeData.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/subscriber.h>
#include <geometry_msgs/Pose2D.h>

#include "trajectory_generator/spline.h"
#include "kinematics/kinematics.h"

using SyncPolicies = message_filters::sync_policies::ApproximateTime<msgs::Path,
                                                                     msgs::VerticeData >;

using Sync = message_filters::Synchronizer<SyncPolicies >;

using Point = Spline::Point;
using Points = Spline::Points;

#define CELL_ROWS 50
#define CELL_COLS 50
#define CELL_SIZE 10

#define SPEED 10 // cm/s

#define DISTANCE_TOLERANCE 5 // cm

class TrajectoryGenerator{
public:
    TrajectoryGenerator();
    ~TrajectoryGenerator();

    void routine();
private:
    ros::NodeHandle nh_;

    message_filters::Subscriber<msgs::Path > path_sub_;
    message_filters::Subscriber<msgs::VerticeData > vertice_sub_;
    void inputUtilsCb(const msgs::PathConstPtr &_path, const msgs::VerticeDataConstPtr &_vertice);
    Sync sync_;
    msgs::Path path_;
    msgs::VerticeData vertice_;

    msgs::MotorVel motor_vel_;
    ros::Publisher motor_vel_pub_;

    ros::Publisher trajectoryx_pub_;
    ros::Publisher trajectoryy_pub_;

    Points knots_;
    Spline spline_x_;
    Spline spline_y_;
    msgs::QuadraticSpline* solutionx;
    msgs::QuadraticSpline* solutiony;
    int piece_wise_idx_;

    void getKnots();

    Point calcReference();

    void process();

    inline int flatIdx(Point _p)const{
        return _p.first + _p.second * CELL_COLS;
    }

    template <typename T> int sgn(T x){
        return x >= 0 ? 1 : -1;
    }

    ros::Publisher robot_pose_pub_;

    double mileage_;
    Point robot_pos_;
    msgs::Odometry odometry_;
    ros::Subscriber odometry_sub_;
    void odometryCb(const msgs::OdometryConstPtr &_msg);
    //---------------------------------------
    //-- PD Controller ----------------------
    //---------------------------------------
    double errorx;
    double errory;
    double prev_errorx;
    double prev_errory;
    double kP;
    double kD;
    //---------------------------------------

};
