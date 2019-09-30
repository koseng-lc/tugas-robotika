#pragma once

#include <msgs/QuadraticSpline.h>

#include <vector>
#include <functional>

#include <armadillo>

#include "trajectory_generator/gauss_seidel.h"

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

    msgs::QuadraticSpline solution_;

public:
    Spline();
    ~Spline();

    inline void addPoint(Point _p){
        points_->push_back(_p);
    }

    inline void clearPoints(){
        points_->clear();
    }

    inline msgs::QuadraticSpline getSolution() const{
        return solution_;
    }

    void solve();

};
