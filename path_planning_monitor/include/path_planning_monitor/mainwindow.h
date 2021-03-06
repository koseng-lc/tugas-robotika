/**
*   @author : koseng (Lintang)
*   @brief : Main Window GUI
*/

#pragma once

#include <QtCore>
#include <QtGui>
#include <QtWidgets>

#include <QMainWindow>

#include <QGroupBox>
#include <QRadioButton>
#include <QGridLayout>
#include <QPushButton>
#include <QThread>
#include <QSpinBox>
#include <QLabel>
#include <QKeyEvent>

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Empty.h>
#include <msgs/MotorVel.h>
#include <msgs/QuadraticSpline.h>

#include <future>

#include <boost/thread.hpp>

#include "path_planning_monitor/ogmview.h"
#include "path_planning_monitor/trajectory_viewer.h"
#include "kinematics/kinematics.h"

#define GUI_SCENE_RATE 60

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private:
    Ui::MainWindow *ui;

private:
    OGMView* ogm_view_;

    QGridLayout* main_layout_;
    QWidget* main_widget_;
    QGroupBox* mode_gb_;
    QGridLayout* mode_layout_;
    QRadioButton* start_rb_;
    QRadioButton* dest_rb_;
    QRadioButton* set_occupancy_rb_;
    QRadioButton* del_occupancy_rb_;

    QGridLayout* misc_layout_;
    QGroupBox* misc_gb_;
    QSpinBox* delay_sb_;
    QLabel* delay_label_;

    QPushButton* solve_pb_;
    QPushButton* reset_robot_pb_;
    QLineEdit* file_name_lined_;
    QPushButton* save_pb_;
    QPushButton* load_pb_;

    void setupWidgets();
    void setupActions();

    QTimer update_timer_;

    void keyPressEvent(QKeyEvent *e);    

    QMenu* view_menu_;
    QList<QAction* > view_menu_list_;
    TrajectoryViewer* trajectory_viewer_;

    //-- ROS

    boost::condition_variable spin_cv_;
    boost::mutex spin_mtx_;
    boost::thread spin_thread_;
    void spinThread();
    ros::Publisher motor_vel_pub_;

    msgs::QuadraticSpline trajectory_;
    ros::Subscriber trajectory_sub_;
    void trajectoryCb(const msgs::QuadraticSplineConstPtr &_msg);

    ros::Publisher reset_robot_pub_;

    //--

private slots:
    void updateScene();
    void modeRBActions();
    void setDelay(int val);
    void resetRobot();
};
