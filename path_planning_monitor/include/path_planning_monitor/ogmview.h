/**
*   @author : koseng (Lintang)
*   @brief : Occupancy Grid Mapping View
*/

#pragma once

#include <QtCore>
#include <QtGui>
#include <QtWidgets>

#include <QGraphicsView>
#include <QGraphicsScene>
#include <QMouseEvent>
#include <QObject>
#include <QTimer>
#include <QThread>
#include <QFuture>
#include <QtConcurrent/QtConcurrent>

#include <ros/ros.h>
#include <ros/package.h>
#include <msgs/GridMapData.h>
#include <msgs/VerticeData.h>
#include <msgs/PlannerInput.h>
#include <msgs/Path.h>

#include <boost/thread/thread.hpp>

#include <fstream>
#include <ostream>
#include <istream>

#define CELL_COLS 50
#define CELL_ROWS 50
#define CELL_SIZE 10

typedef std::pair<int, int > Point;

enum Mode{
    Start,
    Destination,
    SetOccupancy,
    DelOccupancy
};

enum VertexState{
    Visited,
    Unvisited,
    Occupied,
    Unoccupied,
    Solution,
    Source,
    Target,
    Robot
};

class OGMView:public QGraphicsView{
    Q_OBJECT
public:
    OGMView(QWidget* parent=0);
    ~OGMView();

    QGraphicsScene* ogm_scene_;

    static const int VIEW_WIDTH;
    static const int VIEW_HEIGHT;

    void extract();
    void updateScene();
    void setMode(Mode mode);
    inline double& setDelay(){return delay_;}
    inline Point& setStart(){return start_;}
    inline Point& setDest(){return dest_;}

private:
    //-------------------------------------------------------------------------------------------------------
    ros::NodeHandle nh_;

    msgs::GridMapData map_data_;
    msgs::VerticeData vertice_data_;
    ros::Publisher gmd_pub_;
    ros::Publisher vd_pub_;
    void publishMap();
    double delay_;

    ros::Subscriber vd_sub_;
    void verticeDataCb(const msgs::VerticeDataConstPtr& _vd);

    ros::Publisher planner_in_pub_;
    msgs::PlannerInput planner_in_;

    ros::Subscriber path_sub_;
    msgs::Path path_;
    void pathCb(const msgs::PathConstPtr &_path);
    //-------------------------------------------------------------------------------------------------------

    QThread solve_thread_;

    Point start_;
    Point dest_;
    Mode mode_;

    void init();

    QPoint cursor_pos_;
    void mousePressEvent(QMouseEvent* e);
    void mouseMoveEvent(QMouseEvent* e);

    void updateState();

    inline int flatIdx(Point p)const{return p.first + p.second * CELL_COLS;}

    //-------------------------------------------------------------------------------------------------------
    void serialize(std::ofstream* _file, msgs::GridMapData _map);
    void deserialize(std::ifstream& _file, msgs::GridMapData* _map);
    std::string file_name_;

private slots:
    void solve();
    void saveMap();
    void loadMap();
    void setFileName(QString _file_name);

};
