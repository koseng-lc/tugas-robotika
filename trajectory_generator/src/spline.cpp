#include "trajectory_generator/spline.h"

Spline::Spline()
    : points_(new Points)
    , group_(new PolyGroup)
    , solution_(new msgs::QuadraticSpline){

}

Spline::~Spline(){
    delete points_;
    delete group_;
}

void Spline::solve(){
    using GS = GaussSeidel<50>;
    // decrease by one due to a0 already known
    GS solver(3*(points_->size() - 1) - 1);
    GS::Data mcoeff(solver.getCoeffSize());
    GS::Data vconst(solver.getConstSize());

    mcoeff[solver.flatIdx(0,0)] = getX((*points_)[1]);
    mcoeff[solver.flatIdx(1,0)] = 1;
    vconst[0] = getY((*points_)[1]);

    //-- move the row here
    mcoeff[solver.flatIdx(0,1)] = getX(points_->front());
    mcoeff[solver.flatIdx(1,1)] = 1.0;
    vconst[1] = getY(points_->front());
    //--

    int idx(1);
    std::size_t x(2);
    std::size_t y(1);
    std::size_t interior_sz((2 * (points_->size() - 1)) - 3);
    for(std::size_t i(0); i < interior_sz; i+=2,x+=3,idx++){
        ++y;
        mcoeff[solver.flatIdx(x,y)] = getX((*points_)[idx]) * getX((*points_)[idx]);
        mcoeff[solver.flatIdx(x+1,y)] = getX((*points_)[idx]);
        mcoeff[solver.flatIdx(x+2,y)] = 1;
        vconst[y] = getY((*points_)[idx]);

        if(i+1 >= interior_sz)continue;
        ++y;
        mcoeff[solver.flatIdx(x,y)] = getX((*points_)[idx+1]) * getX((*points_)[idx+1]);
        mcoeff[solver.flatIdx(x+1,y)] = getX((*points_)[idx+1]);
        mcoeff[solver.flatIdx(x+2,y)] = 1;
        vconst[y] = getY((*points_)[idx+1]);
    }        

    //the rest of the const vector elements are zero
    ++y;
    idx = 1;
    mcoeff[solver.flatIdx(0, y)] = 1.0;
    //--
    mcoeff[solver.flatIdx(2, y)] = -2.0 * getX((*points_)[idx]);
    mcoeff[solver.flatIdx(3, y)] = -1.0;

    ++y;
    x = 2;
    ++idx;
    for(; y < (solver.getNumVariables()-1); y++){
        mcoeff[solver.flatIdx(x, y)] = 2.0 * getX((*points_)[idx]);
        mcoeff[solver.flatIdx(x+1,y)] = 1.0;

        mcoeff[solver.flatIdx(x+3,y)] = -2.0 * getX((*points_)[idx]);
        mcoeff[solver.flatIdx(x+4,y)] = -1.0;
        x += 3;
    }

    //-- move the row here
    mcoeff[solver.flatIdx(solver.getNumVariables() - 3, y)] = getX(points_->back()) * getX(points_->back());
    mcoeff[solver.flatIdx(solver.getNumVariables() - 2, y)] = getX(points_->back());
    mcoeff[solver.flatIdx(solver.getNumVariables() - 1, y)] = 1.0;
    vconst[y] = getY(points_->back());
    //--

#ifdef DEBUG_SPLINE
    for(std::size_t i(0); i < solver.getCoeffSize(); i++){
        std::cout << "a(" << i%solver.getNumVariables() << i/solver.getNumVariables() << ") : " << mcoeff[i] << std::endl;
    }
#endif

//    solver.initCoeff(mcoeff);
//    solver.initConst(vconst);
//    solver.process();

    arma::mat A(solver.getNumVariables(), solver.getNumVariables());
    arma::vec b(solver.getNumVariables());
    for(std::size_t i(0); i < solver.getNumVariables(); i++){
        b(i) = vconst[i];
        for(std::size_t j(0); j < solver.getNumVariables(); j++){
            A(i,j) = mcoeff[solver.flatIdx(j,i)];
        }
    }

    vec sol(arma::solve(A,b));

    msgs::Quadratic f;
    msgs::QuadraticSpline solution;
    int bound_idx(0);
//    solution.lower_boundx.push_back(getX((*points_)[bound_idx]));
//    solution.lower_boundy.push_back(getY((*points_)[bound_idx]));
//    solution.upper_boundx.push_back(getX((*points_)[bound_idx+1]));
//    solution.upper_boundy.push_back(getY((*points_)[bound_idx+1]));
    solution.upper_bound.push_back(getX((*points_)[bound_idx+1]));
    f.a = 0; f.b = sol(0); f.c = sol(1);
    solution.f.emplace_back(std::move(f));
    for(std::size_t i(2); i < solver.getNumVariables(); i+=3){
        ++bound_idx;
//        solution.lower_boundx.push_back(getX((*points_)[bound_idx]));
//        solution.lower_boundy.push_back(getY((*points_)[bound_idx]));
//        solution.upper_boundx.push_back(getX((*points_)[bound_idx+1]));
//        solution.upper_boundy.push_back(getY((*points_)[bound_idx+1]));
        solution.upper_bound.push_back(getX((*points_)[bound_idx+1]));
        f.a = sol(i);
        f.b = sol(i+1);
        f.c = sol(i+2);
        solution.f.emplace_back(std::move(f));
    }

    *solution_ = solution;

#ifdef DEBUG_SPLINE
    A.print("A : ");
    b.print("b : ");
    sol.print("x : ");
#endif

}

