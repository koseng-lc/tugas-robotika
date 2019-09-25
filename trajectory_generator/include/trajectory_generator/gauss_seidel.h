/**
*   @author ; koseng (Lintang)
*   @brief : Simple implementation of Gauss-Seidel methods
*/

#pragma once

#include <vector>
#include <iostream>
#include <cassert>


template <std::size_t NumVariables, int NumIter = 10>
class GaussSeidel{
public:
    using Data = std::vector<double>;

    GaussSeidel(){
        num_iter = NumIter;
        A.resize(NumVariables * NumVariables);
        x.resize(NumVariables);
        prev_x.resize(NumVariables);
        b.resize(NumVariables);
        error_tolerance = 1e-6;
    }

    void initCoeff(const Data& _A){
        assert(_A.size() == A.size());
        for(std::size_t i(0); i < A.size(); i++){
            A[i] = _A[i];
        }
    }

    void initSolution(const Data& _x){
        assert(_x.size() == x.size());
        for(std::size_t i(0); i < x.size(); i++){
            x[i] = _x[i];
            prev_x[i] = x[i];
        }
    }

    void initConst(const Data& _b){
        assert(_b.size() == b.size());
        for(std::size_t i(0); i < b.size(); i++){
            b[i] = _b[i];
        }
    }

    void setErrorTolerance(double _tol){
        error_tolerance = _tol;
    }

    void process();

    static inline int flatIdx(int i, int j){
        return i + j * static_cast<int>(NumVariables);
    }

    double prevSum(int i, int j);

private:
    Data A;
    Data x;
    Data prev_x;
    Data b;

    int num_iter;
    double error_tolerance;

    double calcError(){
        auto error_sum(.0);
        for(std::size_t i(0); i < x.size(); i++){
            error_sum += (x[i] - prev_x[i]) / x[i];
        }
        return error_sum;
    }

};

template <std::size_t NumVariables, int NumIter>
double GaussSeidel<NumVariables, NumIter>::prevSum(int i, int j){
    auto sum(.0);
    for(std::size_t idx(0); idx < j; idx++){
        sum += A[flatIdx(i, idx)] * x[idx];
    }
    return sum;
}

template <std::size_t NumVariables, int NumIter>
void GaussSeidel<NumVariables, NumIter>::process(){
    std::size_t iter(0);
    do{

        for(std::size_t i(0); i < NumVariables; i++){
            prev_x[i] = x[i];
            x[i] = (1.0 / A[flatIdx(i, i)]) * (b[i] - prevSum(0, i));
            std::cout << "x" << i << " : " << x[i] << std::endl;
        }

        ++iter;

    }while(calcError() > error_tolerance && iter < NumIter);

}
