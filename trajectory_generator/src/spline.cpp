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
    using GS = GaussSeidel<50>;
    // decrease by one due to a0 already known
    GS solver(3*(points_->size() - 1) - 1);
    GS::Data mcoeff(solver.getCoeffSize());
    GS::Data vconst(solver.getConstSize());

    mcoeff[solver.flatIdx(0,0)] = (*points_)[1].first;
    mcoeff[solver.flatIdx(1,0)] = 1;
    vconst[0] = (*points_)[1].second;

    //-- move the row here
    mcoeff[solver.flatIdx(0,1)] = points_->front().first;
    mcoeff[solver.flatIdx(1,1)] = 1.0;
    vconst[1] = points_->front().second;
    //--

    int idx(1);
    std::size_t x(2);
    std::size_t y(1);
    std::size_t interior_sz((2 * (points_->size() - 1)) - 3);
    for(std::size_t i(0); i < interior_sz; i+=2,x+=3,idx++){
        ++y;
        mcoeff[solver.flatIdx(x,y)] = (*points_)[idx].first * (*points_)[idx].first;
        mcoeff[solver.flatIdx(x+1,y)] = (*points_)[idx].first;
        mcoeff[solver.flatIdx(x+2,y)] = 1;
        vconst[y] = (*points_)[idx].second;

        if(i+1 >= interior_sz)continue;
        ++y;
        mcoeff[solver.flatIdx(x,y)] = (*points_)[idx+1].first * (*points_)[idx+1].first;
        mcoeff[solver.flatIdx(x+1,y)] = (*points_)[idx+1].first;
        mcoeff[solver.flatIdx(x+2,y)] = 1;
        vconst[y] = (*points_)[idx+1].second;
    }        

    //the rest of the const vector elements are zero
    ++y;
    std::cout << y << std::endl;
    idx = 1;
    mcoeff[solver.flatIdx(0, y)] = 1.0;
    //--
    mcoeff[solver.flatIdx(2, y)] = -2.0 * (*points_)[idx].first;
    mcoeff[solver.flatIdx(3, y)] = -1.0;

    ++y;
    x = 2;
    ++idx;
    for(; y < (solver.getNumVariables()-1); y++){
        mcoeff[solver.flatIdx(x, y)] = 2.0 * (*points_)[idx].first;
        mcoeff[solver.flatIdx(x+1,y)] = 1.0;

        mcoeff[solver.flatIdx(x+3,y)] = -2.0 * (*points_)[idx].first;
        mcoeff[solver.flatIdx(x+4,y)] = -1.0;
        x+=3;
    }

    //-- move the row here
    mcoeff[solver.flatIdx(solver.getNumVariables() - 3, y)] = points_->back().first * points_->back().first;
    mcoeff[solver.flatIdx(solver.getNumVariables() - 2, y)] = points_->back().first;
    mcoeff[solver.flatIdx(solver.getNumVariables() - 1, y)] = 1.0;
    vconst[y] = points_->back().second;
    //--

    for(std::size_t i(0); i < solver.getCoeffSize(); i++){
        std::cout << "a(" << i%solver.getNumVariables() << i/solver.getNumVariables() << ") : " << mcoeff[i] << std::endl;
    }

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
    solution.lower_bound.push_back((*points_)[bound_idx].first);
    solution.upper_bound.push_back((*points_)[bound_idx+1].first);
    f.a = 0; f.b = sol(0); f.c = sol(1);
    solution.f.emplace_back(std::move(f));
    for(std::size_t i(2); i < solver.getNumVariables(); i+=3){
        ++bound_idx;
        solution.lower_bound.push_back((*points_)[bound_idx].first);
        solution.upper_bound.push_back((*points_)[bound_idx+1].first);
        f.a = sol(i);
        f.b = sol(i+1);
        f.c = sol(i+2);
        solution.f.emplace_back(std::move(f));
    }

    solution_ = solution;

    A.print("A : ");
    b.print("b : ");
    sol.print("x : ");
}

