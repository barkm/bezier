#include "bezier/fit_composite_bezier_curve.h"

#include <bezier/math/tridiagonal.h>

#include <iostream>

namespace bezier {

    using std::vector;
    using Eigen::MatrixXd;
    using Eigen::VectorXd;


    MatrixXd parameterization_matrix(const vector<double> & parameterization, int degree){

        MatrixXd T(parameterization.size(), degree+1);

        for(int i = 0; i < parameterization.size(); i++){
            double t = 1;
            for(int j = 0; j < degree + 1; j++){
                T(i, j) = t;
                t *= parameterization[i];
            }
        }
        return T;
    }

    MatrixXd data_matrix(const vector<VectorXd> & data_points){
        MatrixXd data(data_points.size(), data_points[0].rows());
        for(int i = 0; i < data_points.size(); i++){
            data.block(i, 0, 1, data_points[i].rows()) = data_points[i].transpose();
        }
        return data;
    }


    CompositeBezierCurve fit_composite_bezier_curve(const vector<vector<VectorXd>> & data_points,
                                                    const vector<vector<double>> & parameterization,
                                                    const vector<int> & curve_degrees,
                                                    bool closed_curve){



        unsigned long number_of_curves = curve_degrees.size();

        // set up data matrices

        vector<MatrixXd> data_matrices;
        for (const auto & d : data_points){
            data_matrices.push_back(data_matrix(d));
        }




        // set up parameterization matrices containing (1 t t^2 t^3 ... t^n)

        vector<MatrixXd> t_matrices;
        vector<MatrixXd> t_product_matrices;
        for (int j = 0; j < number_of_curves; ++j) {
            MatrixXd t =parameterization_matrix(parameterization[j], curve_degrees[j]);
            t_matrices.push_back(t);
            t_product_matrices.push_back(t.transpose() * t);

        }

        // set up Q and R matrices

        vector<MatrixXd> q_matrices;
        vector<MatrixXd> r_matrices;

        Eigen::Matrix2d q;
        q << 0, 1,
            -1, 2;

        q_matrices.push_back(bezier_coefficients(curve_degrees[0]));
        r_matrices.push_back(bezier_coefficients(curve_degrees[0]));

        for(int i = 1; i < number_of_curves; i++){
            MatrixXd coefficient_matrix = bezier_coefficients(curve_degrees[i]);
            MatrixXd qprime;
            if(i == 1){
                qprime = MatrixXd::Zero(2, curve_degrees[i-1] + 1); // want to fit all points in first curve
            }
            else{
                qprime = MatrixXd::Zero(2, curve_degrees[i-1] - 1); // want # of cols to be the same as # parameters in
                                                                    // previous curve
            }
            qprime.block(0, qprime.cols()-2, 2, 2) = q;

            q_matrices.push_back(coefficient_matrix.block(0, 0, coefficient_matrix.rows(), 2) * qprime);
            r_matrices.push_back(coefficient_matrix.block(0, 2, coefficient_matrix.rows(), coefficient_matrix.cols() - 2));

        }

        vector<MatrixXd> lower_diagonal, diagonal, upper_diagonal, rhs;

        // diagonal elements
        for(int i = 0; i < number_of_curves - 1; i++){
            MatrixXd diag = r_matrices[i].transpose() * t_product_matrices[i] * r_matrices[i] +
                    q_matrices[i+1].transpose() * t_product_matrices[i+1] * q_matrices[i+1];
            diagonal.push_back(diag);
        }
        diagonal.push_back(r_matrices[number_of_curves-1].transpose() *
                                   t_product_matrices[number_of_curves-1] * r_matrices[number_of_curves-1]);


        // lower diagonal elements
        for(int i = 0; i < number_of_curves - 1; i++){
            lower_diagonal.push_back(r_matrices[i+1].transpose() * t_product_matrices[i+1] * q_matrices[i+1]);
        }

        // upper diagonal elements
        for(int i = 0; i < number_of_curves - 1; i++){
            upper_diagonal.push_back(q_matrices[i+1].transpose() * t_product_matrices[i+1] * r_matrices[i+1]);
        }

        // rhs elements
        for (int i = 0; i < number_of_curves - 1; ++i) {
            rhs.push_back(r_matrices[i].transpose() * t_matrices[i].transpose() * data_matrices[i] +
                          q_matrices[i+1].transpose() * t_matrices[i+1].transpose() * data_matrices[i+1]);
        }
        rhs.push_back(r_matrices[number_of_curves-1].transpose() * t_matrices[number_of_curves-1].transpose()
                      * data_matrices[number_of_curves-1]);


        /*
        std::cout << "Q:" << std::endl;
        for(const auto & s : q_matrices){
            std::cout << s << std::endl;
            std::cout << std::endl;
        }

        std::cout << "R:" << std::endl;
        for(const auto & s : r_matrices){
            std::cout << s << std::endl;
            std::cout << std::endl;
        }

        std::cout << "T:" << std::endl;
        for(const auto & s : t_matrices){
            std::cout << s << std::endl;
            std::cout << std::endl;
        }


        std::cout << "LOWER:" << std::endl;
        for(const auto & s : lower_diagonal){
            std::cout << s << std::endl;
            std::cout << std::endl;
        }

        std::cout << "------" << std::endl;

        std::cout << "DIAGONAL:" << std::endl;
        for(const auto & s : diagonal){
            std::cout << s << std::endl;
            std::cout << std::endl;
        }

        std::cout << "------" << std::endl;

        std::cout << "UPPER:" << std::endl;
        for(const auto & s : upper_diagonal){
            std::cout << s << std::endl;
            std::cout << std::endl;
        }
         */

        vector<MatrixXd> solution = solve_tridiagonal(lower_diagonal, diagonal, upper_diagonal, rhs);

        std::cout << "SOLUTION:" << std::endl;
        for(const auto & s : solution){
            std::cout << s << std::endl;
            std::cout << std::endl;
        }



    }
}

