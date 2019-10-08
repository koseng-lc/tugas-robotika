/**
*   @author : koseng (Lintang)
*   @brief : Simple implementation of Quadratic Spline
*/

#pragma once

#include <msgs/QuadraticSpline.h>

#include <vector>
#include <functional>

#include <armadillo>

#include "trajectory_generator/gauss_seidel.h"

//#define DEBUG_SPLINE

using namespace arma;

class Spline{
public:
    using Poly = std::function<double(double) >;
    using Interval = std::pair<double, double >;
    using Point = std::pair<double, double >;

    struct PieceWise{
      Poly poly;
      Interval interval;
    };

    using Points = std::vector<Point >;
    using PolyGroup = std::vector<PieceWise >;

private:
    Points* points_;
    PolyGroup* group_;

    msgs::QuadraticSpline* solution_;

public:
    Spline();
    ~Spline();

    inline void addPoint(Point _p){
        points_->push_back(_p);
    }

    inline void clearPoints(){
        points_->clear();
    }

    inline msgs::QuadraticSpline* getSolution() const{
        return solution_;
    }

    inline std::size_t pointsSize() const{
        return points_->size();
    }

    static inline double getX(Point p){
        return p.first;
    }

    static inline double getY(Point p){
        return p.second;
    }

    static inline double& setX(Point& p){
        return p.first;
    }

    static inline double& setY(Point& p){
        return p.second;
    }

    static inline double distance(Point p1, Point p2){
        return std::sqrt((getX(p1) - getX(p2))*(getX(p1) - getX(p2)) +
                         (getY(p1) - getY(p2))*(getY(p1) - getY(p2)));
    }

    void solve();

};
