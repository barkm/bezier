#include <bezier/math/tridiagonal.h>

#include <iostream>

namespace bezier {

    vector<MatrixXd> solve_tridiagonal(const vector<MatrixXd> & lower_diagonal,
                                       const vector<MatrixXd> & diagonal,
                                       const vector<MatrixXd> & upper_diagonal,
                                       const vector<MatrixXd> & rhs){
        vector<MatrixXd> upper_diagonal_prime;
        vector<MatrixXd> rhs_prime;
        vector<MatrixXd> solution;

        upper_diagonal_prime.resize(upper_diagonal.size());
        rhs_prime.resize(rhs.size());
        solution.resize(rhs.size());

        MatrixXd upper_prime;

        upper_prime = diagonal[0].inverse() * upper_diagonal[0];
        upper_diagonal_prime[0] = upper_prime;

        for(int i = 1; i < diagonal.size() - 1; i++){
            upper_prime = (diagonal[i] - lower_diagonal[i - 1] * upper_diagonal_prime[i - 1]).inverse()
                          * upper_diagonal[i];
            upper_diagonal_prime[i] = upper_prime;
        }

        MatrixXd rhs_el_prime;

        rhs_el_prime = diagonal[0].inverse() * rhs[0];
        rhs_prime[0] = rhs_el_prime;

        for(int i = 1; i < diagonal.size(); i++){
            rhs_el_prime = (diagonal[i] - lower_diagonal[i - 1] * upper_diagonal_prime[i - 1]).inverse()
                           * (rhs[i] - lower_diagonal[i - 1] * rhs_prime[i - 1]);
            rhs_prime[i] = rhs_el_prime;
        }

        MatrixXd x;
        x = rhs_prime.at(diagonal.size()-1);
        solution[solution.size() - 1] = x;

        for(int i = diagonal.size() - 2; i >= 0; i--){
            x = rhs_prime[i] - upper_diagonal_prime[i] * solution[i+1];
            solution[i] = x;
        }

        return solution;
    }

    vector<MatrixXd> solve_off_tridiagonal(const vector<MatrixXd> & lower_diagonal,
                                           const vector<MatrixXd> & diagonal,
                                           const vector<MatrixXd> & upper_diagonal,
                                           const vector<MatrixXd> & rhs){

        if(diagonal.size() == 2){
            int rows = diagonal[0].rows();
            int cols = diagonal[0].cols();

            MatrixXd A(2 * rows, 2 * cols);
            A.block(0, 0, rows, cols) = diagonal[0];
            A.block(rows, cols, rows, cols) = diagonal[1];

            A.block(0, cols, rows, cols) = lower_diagonal[0];
            A.block(rows, 0, rows, cols) = upper_diagonal[0];

            MatrixXd b(2 * rows, 1);
            b.block(0, 0, rows, 1) = rhs[0];
            b.block(rows, 0, rows, 1) = rhs[1];

            MatrixXd x = A.fullPivHouseholderQr().solve(b);
            return {x.block(0, 0, rows, 1), x.block(rows, 0, rows, 1)};
        }

        vector<MatrixXd> newlower_diagonal, newdiagonal, newupper_diagonal, newrhs;

        MatrixXd A = lower_diagonal.at(0);
        MatrixXd B = diagonal.at(0);
        MatrixXd C = upper_diagonal.at(0);
        MatrixXd d = rhs.at(0);

        if(diagonal.size() == 3){
            newlower_diagonal.push_back(lower_diagonal.at(2) - upper_diagonal.at(2) * B.inverse() * C);
            newupper_diagonal.push_back(upper_diagonal.at(1) - lower_diagonal.at(1) * B.inverse() * A);

        }else{
            newlower_diagonal.push_back(-1*lower_diagonal.at(1) * B.inverse() * A);
            for(int i = 2; i < lower_diagonal.size(); i++){
                newlower_diagonal.push_back(lower_diagonal.at(i));
            }

            for(int i = 1; i < upper_diagonal.size() - 1; i++){
                newupper_diagonal.push_back(upper_diagonal.at(i));
            }
            newupper_diagonal.push_back(-1 * upper_diagonal.at(upper_diagonal.size() - 1) * B.inverse() * C);
        }


        newdiagonal.push_back(diagonal.at(1) - lower_diagonal.at(1) * B.inverse() * C);
        for(int i = 2; i < diagonal.size() - 1; i++){
            newdiagonal.push_back(diagonal.at(i));
        }
        newdiagonal.push_back(diagonal.at(diagonal.size() - 1) - upper_diagonal.at(upper_diagonal.size() - 1) * B.inverse() * A);


        newrhs.push_back(rhs.at(1) - lower_diagonal.at(1) * B.inverse() * d);
        for(int i = 2; i < rhs.size() - 1; i++){
            newrhs.push_back(rhs.at(i));
        }
        newrhs.push_back(rhs.at(rhs.size() - 1) - upper_diagonal.at(upper_diagonal.size() - 1) * B.inverse() * d);


        vector<MatrixXd> xList = solve_off_tridiagonal(newlower_diagonal, newdiagonal, newupper_diagonal, newrhs);

        MatrixXd x = B.inverse()* (d - C * xList.at(0) - A * xList.at(xList.size() - 1));

        xList.insert(xList.begin(), x);

        return xList;




    }
}
