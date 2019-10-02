#pragma once

#include <QDialog>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QGridLayout>

#include <ros/ros.h>
#include <msgs/QuadraticSpline.h>
#include <geometry_msgs/Pose2D.h>

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
    ros::Subscriber solution_sub_;
    msgs::QuadraticSpline solution_;
    void solutionCb(const msgs::QuadraticSplineConstPtr &_msg);

    geometry_msgs::Pose2D robot_pose_;
    ros::Subscriber robot_pose_sub_;
    void robotPoseCb(const geometry_msgs::Pose2DConstPtr &_msg);

    QImage background_;
signals:
    void updateScene();

private slots:
    void updateSceneAct();

};
