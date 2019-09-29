/**
*   @author ; koseng (Lintang)
*   @brief : Simple implementation of Gauss-Seidel methods
*/

#pragma once

#include <vector>
#include <iostream>
#include <cassert>

#define DEBUG_GAUSS_SEIDEL

template <int NumIter = 10>
class GaussSeidel{
public:
    using Data = std::vector<double>;

    GaussSeidel(std::size_t _num_variables){
//        num_iter = NumIter;
        num_variables = _num_variables;
        A.resize(num_variables * num_variables);
        x.resize(num_variables);
        prev_x.resize(num_variables);
        b.resize(num_variables);
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

    inline std::size_t getCoeffSize() const{
        return A.size();
    }

    inline std::size_t getSolutionSize()const{
        return x.size();
    }

    inline std::size_t getConstSize() const{
        return b.size();
    }

    inline std::size_t getNumVariables() const{
        return num_variables;
    }

    void process();

    inline int flatIdx(int i, int j){
        return i + j * static_cast<int>(num_variables);
    }

    double prevSum(std::size_t idx);

private:
    Data A;
    Data x;
    Data prev_x;
    Data b;

//    int num_iter;
    double error_tolerance;
    std::size_t num_variables;

    double calcError(){
        auto max_error(.0);
        auto error(.0);
        for(std::size_t i(0); i < x.size(); i++){
            error = (x[i] - prev_x[i]) / x[i];
            if(error > max_error)
                max_error = error;
        }
        return max_error;
    }

};

template <int NumIter>
double GaussSeidel<NumIter>::prevSum(std::size_t idx){
    auto sum(.0);
    for(std::size_t i(0); i < num_variables; i++){
        if(i != idx)
            sum += A[flatIdx(i, idx)] * x[i];
    }
    return sum;
}

template <int NumIter>
void GaussSeidel<NumIter>::process(){
    std::size_t iter(0);
    auto error(.0);
    do{

        for(std::size_t i(0); i < num_variables; i++){
            prev_x[i] = x[i];
            x[i] = (1.0 / A[flatIdx(i, i)]) * (b[i] - prevSum(i));
#ifdef DEBUG_GAUSS_SEIDEL
            std::cout << "x" << i << " : " << x[i] << " ; prev : " << prev_x[i] << std::endl;
#endif
        }

        ++iter;
        error = calcError();
#ifdef DEBUG_GAUSS_SEIDEL
        std::cout << "Error after " << iter << " iterations : " << error << std::endl;
#endif
    }while(error > error_tolerance && iter < NumIter);

}
