#ifndef BEZIER_TRIDIAGONAL_H
#define BEZIER_TRIDIAGONAL_H

#include <vector>

#include <Eigen/Dense>

namespace bezier {
    using std::vector;
    using Eigen::MatrixXd;

    vector<MatrixXd> solve_tridiagonal(const vector<MatrixXd> & lower_diagonal,
                                       const vector<MatrixXd> & diagonal,
                                       const vector<MatrixXd> & upper_diagonal,
                                       const vector<MatrixXd> & rhs);


    vector<MatrixXd> solve_off_tridiagonal(const vector<MatrixXd> & lower_diagonal,
                                           const vector<MatrixXd> & diagonal,
                                           const vector<MatrixXd> & upper_diagonal,
                                           const vector<MatrixXd> & rhs);

}


#endif //BEZIER_TRIDIAGONAL_H
