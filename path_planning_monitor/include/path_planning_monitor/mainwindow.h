#ifndef MAINWINDOW_H
#define MAINWINDOW_H

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
#include <msgs/MotorVel.h>
#include <msgs/QuadraticSpline.h>

#include <future>

#include <boost/thread.hpp>

#include "ogmview.h"
#include "kinematics/kinematics.h"

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

    void setupWidgets();
    void setupActions();

    QTimer update_timer_;

    void keyPressEvent(QKeyEvent *e);

    boost::condition_variable spin_cv_;
    boost::mutex spin_mtx_;
    boost::thread spin_thread_;
    void spinThread();
    ros::Publisher motor_vel_pub_;

    msgs::QuadraticSpline trajectory_;
    ros::Subscriber trajectory_sub_;
    void trajectoryCb(const msgs::QuadraticSplineConstPtr &_msg);

private slots:
    void updateScene();
    void modeRBActions();
    void setDelay(int val);
};

#endif // MAINWINDOW_H
