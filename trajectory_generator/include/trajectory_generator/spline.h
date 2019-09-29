#pragma once

#include <vector>
#include <functional>

#include "trajectory_generator/gauss_seidel.h"

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

public:
    Spline();
    ~Spline();

    inline void addPoint(Point _p){
        points_->push_back(_p);
    }

    inline void clearPoints(){
        points_->clear();
    }

    void solve();

};
