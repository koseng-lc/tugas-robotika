#include "trajectory_generator/spline.h"

Spline::Spline()
    : points_(new Points)
    , group_(new PolyGroup){

}

Spline::~Spline(){
    delete points_;
    delete group_;
}

void Spline::solve(){

}
