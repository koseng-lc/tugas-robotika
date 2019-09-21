#include "path_planning_monitor/mainwindow.h"
#include <QApplication>

#include <ros/ros.h>

#include <csignal>

void sigHandler(int sig){
    (void)sig;
    QApplication::quit();
}

int main(int argc, char *argv[]){

    ros::init(argc, argv, "path_planning_monitor_node", ros::InitOption::NoSigintHandler);

    signal(SIGINT, sigHandler);

    QApplication a(argc, argv);
    MainWindow w;
    w.show();

    return a.exec();
}
