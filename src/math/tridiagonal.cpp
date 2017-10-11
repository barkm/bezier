#include <bezier/math/tridiagonal.h>

#include <iostream>

namespace bezier {

    std::string _invalid_number_of_elements_message(const vector<MatrixXd> & lower_diagonal,
                                       const vector<MatrixXd> & diagonal,
                                       const vector<MatrixXd> & upper_diagonal,
                                       const vector<MatrixXd> & rhs) {
        return std::string("Invalid number of block matrix elements:"
                                   " diagonal: " + std::to_string(diagonal.size()) +
                           ", lower: " + std::to_string(lower_diagonal.size()) +
                           ", upper: " + std::to_string(upper_diagonal.size()) +
                           ", rhs: " + std::to_string(rhs.size()));
    }

    vector<MatrixXd> solve_tridiagonal(const vector<MatrixXd> & lower_diagonal,
                                        const vector<MatrixXd> & diagonal,
                                        const vector<MatrixXd> & upper_diagonal,
                                        const vector<MatrixXd> & rhs) {

        if(diagonal.empty()){
            throw std::invalid_argument(_invalid_number_of_elements_message(lower_diagonal,
                                                                            diagonal,
                                                                            upper_diagonal,
                                                                            rhs));
        }

        if(diagonal.size() != lower_diagonal.size() + 1 or
           diagonal.size() != upper_diagonal.size() + 1 or
           diagonal.size() != rhs.size()){
            throw std::invalid_argument(_invalid_number_of_elements_message(lower_diagonal,
                                                                            diagonal,
                                                                            upper_diagonal,
                                                                            rhs));
        }

        // special case if there is only one element on the diagonal
        if(diagonal.size() == 1 and rhs.size() == 1
           and lower_diagonal.size() == 0 and upper_diagonal.size() == 0){
            return _solve_tridiagonal(lower_diagonal, diagonal, upper_diagonal, rhs);
        }

        // upper left corner
        if(diagonal[0].cols() != lower_diagonal[0].cols() or diagonal[0].rows() != upper_diagonal[0].rows() or
                diagonal[0].rows() != rhs[0].rows()){
            throw std::invalid_argument("Invalid matrix dimensions");
        }

        // middle
        for(int i = 1; i < diagonal.size()-1; i++){
            if(diagonal[i].cols() != upper_diagonal[i-1].cols() or
               diagonal[i].cols() != lower_diagonal[i].cols() or
               diagonal[i].rows() != upper_diagonal[i].rows() or
               diagonal[i].rows() != lower_diagonal[i-1].rows() or
               diagonal[i].rows() != rhs[i].rows()){
                throw std::invalid_argument("Invalid matrix dimensions");
            }
        }

        // bottom right corner
        if(diagonal[diagonal.size()-1].cols() != upper_diagonal[upper_diagonal.size()-1].cols() or
                diagonal[diagonal.size()-1].rows() != lower_diagonal[lower_diagonal.size()-1].rows() or
                diagonal[diagonal.size()-1].rows() != rhs[rhs.size()-1].rows()){
            throw std::invalid_argument("Invalid matrix dimensions");
        }

        // check that all rhs matrices have same number of columns
        long col_size = rhs[0].cols();
        for (const auto & r : rhs) {
            if (r.cols() != col_size){
                throw std::invalid_argument("Invalid matrix dimensions. All right hand side matrices must"
                                                    " have the same number of columns.");
            }
        }

        return _solve_tridiagonal(lower_diagonal, diagonal, upper_diagonal, rhs);
    }


    vector<MatrixXd> _solve_tridiagonal(const vector<MatrixXd> & lower_diagonal,
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


        for(int i = 0; i < diagonal.size() - 1; i++){
            if(i == 0){
                upper_prime = diagonal[0].inverse() * upper_diagonal[0];
                upper_diagonal_prime[0] = upper_prime;
            }
            else{
                upper_prime = (diagonal[i] - lower_diagonal[i - 1] * upper_diagonal_prime[i - 1]).inverse()
                              * upper_diagonal[i];
                upper_diagonal_prime[i] = upper_prime;
            }
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

        if(diagonal.empty()){
            throw std::invalid_argument(_invalid_number_of_elements_message(lower_diagonal,
                                                                            diagonal,
                                                                            upper_diagonal,
                                                                            rhs));
        }

        if(diagonal.size() == 1){
            if(!lower_diagonal.empty() or !upper_diagonal.empty() or rhs.size() != 1){
                throw std::invalid_argument(_invalid_number_of_elements_message(lower_diagonal,
                                                                                diagonal,
                                                                                upper_diagonal,
                                                                                rhs));
            }
        }
        else if(diagonal.size() == 2){
            if(lower_diagonal.size() != 1 or upper_diagonal.size() != 1 or rhs.size() != 2){
                throw std::invalid_argument(_invalid_number_of_elements_message(lower_diagonal,
                                                                                diagonal,
                                                                                upper_diagonal,
                                                                                rhs));
            }

            // upper left corner
            if(
                    diagonal[0].cols() != lower_diagonal[0].cols() or
                    diagonal[0].rows() != upper_diagonal[0].rows() or
                    diagonal[0].rows() != rhs[0].rows()
                    )
            {
                throw std::invalid_argument("Invalid matrix dimensions");
            }

            // bottom right corner
            if(diagonal[1].cols() != upper_diagonal[0].cols() or
               diagonal[1].rows() != lower_diagonal[0].rows() or
               diagonal[1].rows() != rhs[1].rows()){
                throw std::invalid_argument("Invalid matrix dimensions");
            }
        }
        else{

            if(diagonal.size() != lower_diagonal.size() or
               diagonal.size() != upper_diagonal.size() or
               diagonal.size() != rhs.size()){
                throw std::invalid_argument("Invalid number of block matrix elements");
            }

            unsigned long size = diagonal.size();

            // upper left corner
            if(
                    diagonal[0].cols() != lower_diagonal[1].cols() or
                    diagonal[0].cols() != upper_diagonal[size-1].cols() or
                    diagonal[0].rows() != upper_diagonal[0].rows() or
                    diagonal[0].rows() != lower_diagonal[0].rows() or
                    diagonal[0].rows() != rhs[0].rows()
                    )
            {
                throw std::invalid_argument("Invalid matrix dimensions");
            }

            // middle
            for(int i = 1; i < diagonal.size()-1; i++){
                if(diagonal[i].cols() != upper_diagonal[i-1].cols() or
                   diagonal[i].cols() != lower_diagonal[i+1].cols() or
                   diagonal[i].rows() != upper_diagonal[i].rows() or
                   diagonal[i].rows() != lower_diagonal[i].rows() or
                   diagonal[i].rows() != rhs[i].rows()){
                    throw std::invalid_argument("Invalid matrix dimensions");
                }
            }

            // bottom right corner
            if(diagonal[size-1].cols() != upper_diagonal[size-2].cols() or
               diagonal[size-1].cols() != lower_diagonal[0].cols() or
               diagonal[size-1].rows() != lower_diagonal[size-1].rows() or
               diagonal[size-1].rows() != upper_diagonal[size-1].rows() or
               diagonal[size-1].rows() != rhs[size-1].rows()){
                throw std::invalid_argument("Invalid matrix dimensions");
            }

        }

        // check that all rhs matrices have same number of columns
        long col_size = rhs[0].cols();
        for (const auto & r : rhs) {
            if (r.cols() != col_size){
                throw std::invalid_argument("Invalid matrix dimensions. All right hand side matrices must"
                                                    " have the same number of columns.");
            }
        }

        return _solve_off_tridiagonal(lower_diagonal, diagonal, upper_diagonal, rhs);
    }


    vector<MatrixXd> _solve_off_tridiagonal(const vector<MatrixXd> & lower_diagonal,
                                           const vector<MatrixXd> & diagonal,
                                           const vector<MatrixXd> & upper_diagonal,
                                           const vector<MatrixXd> & rhs){

        if(diagonal.size() == 1){
            MatrixXd x = diagonal[0].fullPivHouseholderQr().solve(rhs[0]);
            return {x};
        }
        if(diagonal.size() == 2){
            MatrixXd A(diagonal[0].rows() + diagonal[1].rows(), diagonal[0].cols() + diagonal[1].cols());
            A.block(0, 0, diagonal[0].rows(), diagonal[0].cols()) = diagonal[0];
            A.block(diagonal[0].rows(), diagonal[0].cols(), diagonal[1].rows(), diagonal[1].cols()) = diagonal[1];

            A.block(0, diagonal[0].cols(), upper_diagonal[0].rows(), upper_diagonal[0].cols()) = upper_diagonal[0];
            A.block(diagonal[0].rows(), 0, lower_diagonal[0].rows(), lower_diagonal[0].cols()) = lower_diagonal[0];

            MatrixXd b(rhs[0].rows() + rhs[1].rows(), rhs[0].cols());
            b.block(0, 0, rhs[0].rows(), rhs[0].cols()) = rhs[0];
            b.block(rhs[0].rows(), 0, rhs[1].rows(), rhs[1].cols()) = rhs[1];

            MatrixXd x = _solve_off_tridiagonal({}, {A}, {}, {b})[0];

            return {x.block(0, 0, rhs[0].rows(), rhs[0].cols()), x.block(rhs[0].rows(), 0, rhs[1].rows(), rhs[1].cols())};
        }

        vector<MatrixXd> newlower_diagonal, newdiagonal, newupper_diagonal, newrhs;

        MatrixXd A = lower_diagonal[0];
        MatrixXd B = diagonal[0];
        MatrixXd C = upper_diagonal[0];
        MatrixXd d = rhs[0];

        if(diagonal.size() == 3){
            newlower_diagonal.push_back(lower_diagonal[2] - upper_diagonal[2] * B.inverse() * C);
            newupper_diagonal.push_back(upper_diagonal[1] - lower_diagonal[1] * B.inverse() * A);

        }else{
            newlower_diagonal.push_back(-1*lower_diagonal[1] * B.inverse() * A);
            for(int i = 2; i < lower_diagonal.size(); i++){
                newlower_diagonal.push_back(lower_diagonal[i]);
            }

            for(int i = 1; i < upper_diagonal.size() - 1; i++){
                newupper_diagonal.push_back(upper_diagonal[i]);
            }
            newupper_diagonal.push_back(-1 * upper_diagonal[upper_diagonal.size() - 1] * B.inverse() * C);
        }


        newdiagonal.push_back(diagonal[1] - lower_diagonal[1] * B.inverse() * C);
        for(int i = 2; i < diagonal.size() - 1; i++){
            newdiagonal.push_back(diagonal[i]);
        }
        newdiagonal.push_back(diagonal[diagonal.size() - 1] - upper_diagonal[upper_diagonal.size() - 1] * B.inverse() * A);


        newrhs.push_back(rhs[1] - lower_diagonal[1] * B.inverse() * d);
        for(int i = 2; i < rhs.size() - 1; i++){
            newrhs.push_back(rhs[i]);
        }
        newrhs.push_back(rhs[rhs.size() - 1] - upper_diagonal[upper_diagonal.size() - 1] * B.inverse() * d);


        vector<MatrixXd> xList = solve_off_tridiagonal(newlower_diagonal, newdiagonal, newupper_diagonal, newrhs);

        MatrixXd x = B.inverse()* (d - C * xList[0] - A * xList[xList.size() - 1]);

        xList.insert(xList.begin(), x);

        return xList;




    }
}
