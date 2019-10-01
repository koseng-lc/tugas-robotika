#ifndef OGMVIEW_H
#define OGMVIEW_H

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
#include <msgs/GridMapData.h>
#include <msgs/VerticeData.h>
#include <msgs/PlannerInput.h>
#include <msgs/Path.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>

#include <boost/thread/thread.hpp>

#define CELL_COLS 50
#define CELL_ROWS 50
#define CELL_SIZE 10

typedef message_filters::sync_policies::ApproximateTime<msgs::GridMapData,
                                                        msgs::VerticeData > SyncPolicy;
typedef message_filters::Synchronizer<SyncPolicy > Sync;

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
    OGMView();
    ~OGMView();

    QGraphicsScene* ogm_scene_;

    static const int MAP_WIDTH;
    static const int MAP_HEIGHT;

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

    message_filters::Subscriber<msgs::GridMapData > gmd_sub_;
    message_filters::Subscriber<msgs::VerticeData > vd_sub_;
    void inputUtilsCb(const msgs::GridMapDataConstPtr& _gmd, const msgs::VerticeDataConstPtr& _vd);

    Sync sync_;

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

private slots:
    void solve();

};

#endif // OGMVIEW_H
