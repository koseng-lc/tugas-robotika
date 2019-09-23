#pragma once

#include <ros/ros.h>
#include <msgs/MotorVel.h>
#include <msgs/Path.h>
#include <msgs/VerticeData.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/subscriber.h>

#include "trajectory_generator/spline.h"

using SyncPolicies = message_filters::sync_policies::ApproximateTime<msgs::Path,
                                                                     msgs::VerticeData >;

using Sync = message_filters::Synchronizer<SyncPolicies >;

using Point = Spline::Point;
using Points = Spline::Points;

#define CELL_ROWS 60
#define CELL_COLS 90
#define CELL_SIZE 10

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

    ros::Publisher motor_vel_pub_;

    Points knots_;
    Spline spline_;

    void getKnots();

    void process();

    inline int flatIdx(Point _p)const{
        return _p.first + _p.second * CELL_COLS;
    }

};
