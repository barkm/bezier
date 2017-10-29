#ifndef BEZIER_TRIDIAGONAL_H
#define BEZIER_TRIDIAGONAL_H

#include <vector>

#include <Eigen/Dense>

namespace bezier {
    using std::vector;
    using Eigen::MatrixXd;

    /**
     * Solve tri-diagonal block matrix system on the form
     *
     * |  B_1  C_1  0    0    0     ...     0     | | X_1   |     | D_1   |
     * |  A_1  B_2  C_2  0    0     ...     0     | | X_2   |     | D_2   |
     * |  0    A_2  B_3  C_3  0     ...     0     | |       |     |       |
     * |                                          | |  .    |     |  .    |
     * |  .           .    .    .           .     | |  .    |  =  |  .    |
     * |  .             .    .    .         .     | |  .    |     |  .    |
     * |  .               .    .    .       .     | |       |     |       |
     * |                      A_n-2  B_n-1  C_n-1 | | X_n-1 |     | D_n-1 |
     * |  0                   0       A_n-1 B_n   | | X_n   |     | D_n   |
     *
     * @param lower_diagonal : list of matrices on the lower diagonal {A_1, ..., A_n-1}
     * @param diagonal : list of matrices on the diagonal {B_0, ..., B_n}
     * @param upper_diagonal : list of matrices on the upper diagonal {C_1, ..., C_n-1}
     * @param rhs : list of matrices on the right and side {D_1, ..., D_n}
     * @return list of solutions {X_1, ..., X_n}
     */
    vector<MatrixXd> solve_tridiagonal(const vector<MatrixXd> & lower_diagonal,
                                       const vector<MatrixXd> & diagonal,
                                       const vector<MatrixXd> & upper_diagonal,
                                       const vector<MatrixXd> & rhs);

    vector<MatrixXd> _solve_tridiagonal(const vector<MatrixXd> & lower_diagonal,
                                       const vector<MatrixXd> & diagonal,
                                       const vector<MatrixXd> & upper_diagonal,
                                       const vector<MatrixXd> & rhs);

    /**
     * Solve tri-diagonal block matrix system on the form
     *
     * |  B_1  C_2  0    0    0     ...     A_1 | | X_1   |     | D_1   |
     * |  A_2  B_2  C_3  0    0     ...     0   | | X_2   |     | D_2   |
     * |  0    A_3  B_3  C_4  0     ...     0   | |       |     |       |
     * |                                        | |  .    |     |  .    |
     * |  .           .    .    .           .   | |  .    |  =  |  .    |
     * |  .             .    .    .         .   | |  .    |     |  .    |
     * |  .               .    .    .       .   | |       |     |       |
     * |                      A_n-1  B_n-1  C_n | | X_n-1 |     | D_n-1 |
     * |  C_1                 0      A_n    B_n | | X_n   |     | D_n   |
     *
     * @param lower_diagonal : list of matrices on the lower diagonal {A_1, ..., A_n}
     * @param diagonal : list of matrices on the diagonal {B_1, ..., B_n}
     * @param upper_diagonal : list of matrices on the upper diagonal {C_1, ..., C_n}
     * @param rhs : list of matrices on the right and side {D_1, ..., D_n}
     * @return list of solutions {X_1, ..., X_n}
     */
    vector<MatrixXd> solve_off_tridiagonal(const vector<MatrixXd> & lower_diagonal,
                                           const vector<MatrixXd> & diagonal,
                                           const vector<MatrixXd> & upper_diagonal,
                                           const vector<MatrixXd> & rhs);

    vector<MatrixXd> _solve_off_tridiagonal(const vector<MatrixXd> & lower_diagonal,
                                           const vector<MatrixXd> & diagonal,
                                           const vector<MatrixXd> & upper_diagonal,
                                           const vector<MatrixXd> & rhs);

}


#endif //BEZIER_TRIDIAGONAL_H
