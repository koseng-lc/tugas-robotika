/**
*   @author : koseng (Lintang)
*   @brief : Abstract class for path planning solver
*/

#pragma once

#include <queue>
#include <stack>
#include <iostream>

#include <boost/graph/adjacency_list.hpp>

#include "path_planning/ogm.h"

#define CELL_COLS 50
#define CELL_ROWS 50
#define CELL_SIZE 10

typedef std::pair<int, int > Point;

enum VertexState{
    Visited,
    Unvisited,
    Occupied,
    Unoccupied,
    Solution,
    Source,
    Target
};

struct EdgeProp{
    double dist;
};

struct VertexProp{
    int x;
    int y;
    int prev_idx;
    double total_dist;
    VertexState state;
};

typedef boost::adjacency_list<boost::setS, boost::listS, boost::undirectedS,
        VertexProp, EdgeProp > Graph;
typedef boost::graph_traits<Graph >::vertex_descriptor Vertex;
typedef boost::graph_traits<Graph >::edge_descriptor Edge;

class Solver{
public:
    Solver();
    virtual ~Solver();

    OGM* getOGM() const{
        return ogm_;
    }

    Graph* getGraph(){
        return graph_;
    }

    inline bool isFinished() const{return finished_;}
    inline int& setDelay(){return delay_;}
    inline int getDelay() const{return delay_;}
    inline Point& setSource(){return source_;}
    inline Point& setTarget(){return target_;}

    static inline int flatIdx(int x, int y){return (y * CELL_COLS + x);}
    static inline int getX(Point _p){
        return _p.first;
    }

    static inline int getY(Point _p){
        return _p.second;
    }

    static inline int& setX(Point& _p){
        return _p.first;
    }

    static inline int& setY(Point& _p){
        return _p.second;
    }

    virtual void solve() = 0;
    virtual void solvePerStep() = 0;
    virtual void reinit() = 0;
protected:
    Point source_;
    Point target_;

    bool finished_;
    int delay_;

    Graph* graph_;
    OGM* ogm_;

private:
    void initGraph();

};

