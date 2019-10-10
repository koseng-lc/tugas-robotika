/**
*   @author : koseng (Lintang)
*   @brief : Trajectory Viewer
*/

#pragma once

#include <QDialog>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QGridLayout>

#include <ros/ros.h>
#include <msgs/QuadraticSpline.h>
#include <geometry_msgs/Pose2D.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/subscriber.h>

using SyncPolicy2 = message_filters::sync_policies::ApproximateTime<msgs::QuadraticSpline,
                                                                    msgs::QuadraticSpline>;

using Sync2 = message_filters::Synchronizer<SyncPolicy2>;

class TrajectoryViewer:public QDialog{
    Q_OBJECT
public:
    TrajectoryViewer(QWidget* parent = 0);
    ~TrajectoryViewer();

private:
    QGraphicsScene* scene_;
    QGraphicsView* view_;
    QGridLayout* main_layout_;

    ros::NodeHandle g_nh_;

    message_filters::Subscriber<msgs::QuadraticSpline > solx_sub_;
    message_filters::Subscriber<msgs::QuadraticSpline > soly_sub_;
    Sync2 sync_;
    void solutionCb(const msgs::QuadraticSplineConstPtr& _msgx,
                    const msgs::QuadraticSplineConstPtr& _msgy);
    msgs::QuadraticSpline solx_;
    msgs::QuadraticSpline soly_;

    geometry_msgs::Pose2D robot_pose_;
    ros::Subscriber robot_pose_sub_;
    void robotPoseCb(const geometry_msgs::Pose2DConstPtr &_msg);

    QImage background_;

    //--temporary
    std::vector<geometry_msgs::Pose2D > robot_trajectory_;

signals:
    void updateScene();

private slots:
    void updateSceneAct();

};
