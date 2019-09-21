#pragma once

#include <boost/numeric/ublas/matrix.hpp>

typedef boost::numeric::ublas::matrix<double> Matrix;
typedef boost::numeric::ublas::zero_matrix<double> Zeros;

class OGM{
public:
    OGM(int _w, int _h);
    Matrix map_;

private:

};
