#include "bezier/fit_composite_bezier_curve.h"

#include <bezier/math/tridiagonal.h>
#include <bezier/utilities.h>

#include <iostream>

namespace bezier {

    using std::vector;
    using Eigen::MatrixXd;
    using Eigen::VectorXd;


    vector<double> chordlength_parameterization(const vector<VectorXd> &data_points,
                                                const VectorXd &start_point){
        // get total length
        double total_length = 0;
        if(start_point.rows() != 0){
            total_length += (data_points[0]-start_point).norm();
        }

        for(int i = 1; i < data_points.size(); i++){
            total_length += (data_points[i] - data_points[i-1]).norm();
        }

        // find parameterization
        vector<double> parameterization;
        double length = 0;
        if(start_point.rows() != 0) {
            length += (data_points[0] - start_point).norm();
        }

        parameterization.push_back(length / total_length);
        for(int i = 1; i < data_points.size(); i++){
            length += (data_points[i] - data_points[i-1]).norm();
            parameterization.push_back(length / total_length);
        }

        return parameterization;
    }

    vector<vector<double>> initialize_parameterization(const vector<vector<VectorXd>> & data_points, bool closed_curve){

        for(const auto & data : data_points){
            if(data.empty()){
                throw std::invalid_argument("Cannot have empty data point partition!");
            }
        }

        vector<vector<double>> parameterization;
        for(int i = 0; i < data_points.size(); i++){
            if(i != 0){
                parameterization.push_back(chordlength_parameterization(data_points[i],
                                                                        *(data_points[i - 1].end() - 1)));
            }
            else{
                if(closed_curve){
                    parameterization.push_back(
                            chordlength_parameterization(data_points[i],
                                                         *(data_points[data_points.size() - 1].end() - 1)));
                }
                else{
                    parameterization.push_back(chordlength_parameterization(data_points[i]));
                }
            }
        }
        return parameterization;
    }


    CompositeBezierCurve fit_composite_bezier_curve(const vector<VectorXd> & data_points,
                                                    const vector<int> & joints,
                                                    int curve_degrees,
                                                    bool closed_curve){
        vector<int> curve_degrees_vec(joints.size()+1, curve_degrees);
        return fit_composite_bezier_curve(data_points, joints, curve_degrees_vec, closed_curve);
    }


    CompositeBezierCurve fit_composite_bezier_curve(const vector<VectorXd> & data_points,
                                                    const vector<int> & joints,
                                                    const vector<int> & curve_degrees,
                                                    bool closed_curve){
        return fit_composite_bezier_curve(partition_data<VectorXd>(data_points, joints),
                                          curve_degrees, closed_curve);
    }


    CompositeBezierCurve fit_composite_bezier_curve(const vector<vector<VectorXd>> & data_points,
                                                    const vector<int> & curve_degrees,
                                                    bool closed_curve){
        return fit_composite_bezier_curve(data_points,
                                          initialize_parameterization(data_points, closed_curve),
                                          curve_degrees,
                                          closed_curve);
    }


    MatrixXd parameterization_matrix(const vector<double> &parameterization, int degree){
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


    void _argument_check_fit_composite_bezier_curve(const vector<vector<VectorXd>> & data_points,
                                                   const vector<vector<double>> & parameterization,
                                                   const vector<int> & curve_degrees,
                                                   bool closed_curve) {

        if(parameterization.size() != data_points.size()){
           throw std::invalid_argument("Number of parameterizations does not match number of curves");
        }

        // keeps track if the previous curve has data point assigned parameterization = 1
        bool prev_curve_has_end_point = false;
        if(closed_curve){
            for (auto t : parameterization[parameterization.size()-1]){
                if(t == 1){
                    prev_curve_has_end_point = true;
                }
            }
        }

        for (int i = 0; i < data_points.size(); ++i) {
            if(i == 0 and !closed_curve){
                if(data_points[i].size() < curve_degrees[i] + 1){
                    throw std::invalid_argument("Not enough data points to fit curve.");
                }
                if(curve_degrees[i] < 2 and data_points.size() != 1){
                    throw std::invalid_argument("First bezier curve must be of degree at least 2.");
                }
            }
            else{
                if(data_points[i].size() < curve_degrees[i] - 1){
                    throw std::invalid_argument("Not enough data points to fit curve.");
                }
                if(curve_degrees[i] < 3){
                    throw std::invalid_argument("The Bezier curves must be of degree at least 3.");
                }
            }

            if(data_points[i].size() != parameterization[i].size()){
                throw std::invalid_argument("Number of parameterization parameters and data points must be the same.");
            }

            bool has_end_point = false;
            for (auto t : parameterization[i]){
                if(t == 0 and prev_curve_has_end_point){
                    throw std::invalid_argument("Ambiguous parameterization.");
                }
                if(t == 1){
                    has_end_point = true;
                }
                if(t < 0 or t > 1){
                    throw std::invalid_argument("Parameterization parameters must be in range [0, 1]");
                }
            }
            prev_curve_has_end_point = has_end_point;
        }
    }

    CompositeBezierCurve fit_composite_bezier_curve(const vector<vector<VectorXd>> & data_points,
                                                    const vector<vector<double>> & parameterization,
                                                    const vector<int> & curve_degrees,
                                                    bool closed_curve){

        // check valid arguments
        _argument_check_fit_composite_bezier_curve(data_points, parameterization, curve_degrees, closed_curve);


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
            MatrixXd t = parameterization_matrix(parameterization[j], curve_degrees[j]);
            t_matrices.push_back(t);
            t_product_matrices.push_back(t.transpose() * t);

        }

        // set up Q and R matrices

        vector<MatrixXd> q_matrices;
        vector<MatrixXd> continuity_matrices;
        vector<MatrixXd> r_matrices;

        Eigen::Matrix2d q;
        q << 0, 1, -1, 2;

        for(int i = 0; i < number_of_curves; i++){
            MatrixXd coefficient_matrix = bezier_coefficients(curve_degrees[i]);
            MatrixXd continuity_matrix;

            if(i == 0 and !closed_curve){
                q_matrices.push_back(coefficient_matrix);
                r_matrices.push_back(coefficient_matrix);
            }
            else{
                if(i == 0){ // if first curve in closed composite curve
                    continuity_matrix = MatrixXd::Zero(2, curve_degrees[number_of_curves-1] - 1);
                }
                else if(i == 1 and !closed_curve){
                    continuity_matrix = MatrixXd::Zero(2, curve_degrees[i-1] + 1); // want to fit all points in first curve
                }
                else{
                    continuity_matrix = MatrixXd::Zero(2, curve_degrees[i-1] - 1);  // want # of cols to be the same as
                                                                                    // # parameters in previous curve
                }
                continuity_matrix.block(0, continuity_matrix.cols()-2, 2, 2) = q;

                q_matrices.push_back(coefficient_matrix.block(0, 0, coefficient_matrix.rows(), 2) * continuity_matrix);
                r_matrices.push_back(coefficient_matrix.block(0, 2, coefficient_matrix.rows(), coefficient_matrix.cols() - 2));

                continuity_matrices.push_back(continuity_matrix);
            }
        }

        vector<MatrixXd> lower_diagonal, diagonal, upper_diagonal, rhs;

        // diagonal elements
        MatrixXd diag;
        for(int i = 0; i < number_of_curves - 1; i++){
            diag = r_matrices[i].transpose() * t_product_matrices[i] * r_matrices[i] +
                    q_matrices[i+1].transpose() * t_product_matrices[i+1] * q_matrices[i+1];
            diagonal.push_back(diag);
        }
        diag = r_matrices[number_of_curves-1].transpose()
               * t_product_matrices[number_of_curves-1] * r_matrices[number_of_curves-1];
        if(closed_curve) {
            diag += q_matrices[0].transpose() * t_product_matrices[0] * q_matrices[0];
        }
        diagonal.push_back(diag);



        // lower diagonal elements
        if(closed_curve){
            if(number_of_curves == 1){
                diagonal[0] += r_matrices[0].transpose() * t_product_matrices[0] * q_matrices[0];
            }
            else {
                lower_diagonal.push_back(r_matrices[0].transpose() * t_product_matrices[0] * q_matrices[0]);
            }
        }
        for(int i = 0; i < number_of_curves - 1; i++){
            lower_diagonal.push_back(r_matrices[i+1].transpose() * t_product_matrices[i+1] * q_matrices[i+1]);
        }

        // upper diagonal elements
        for(int i = 0; i < number_of_curves - 1; i++){
            upper_diagonal.push_back(q_matrices[i+1].transpose() * t_product_matrices[i+1] * r_matrices[i+1]);
        }
        if(closed_curve){
            if(number_of_curves == 1){
                diagonal[0] += q_matrices[0].transpose() * t_product_matrices[0] * r_matrices[0];
            }
            else{
                upper_diagonal.push_back(q_matrices[0].transpose() * t_product_matrices[0] * r_matrices[0]);
            }
        }


        // if closed and # of curves == 2 make modifications to matrix system
        if(closed_curve and number_of_curves == 2){
            lower_diagonal[1] += upper_diagonal[1];
            upper_diagonal[0] += lower_diagonal[0];

            lower_diagonal.erase(lower_diagonal.begin());
            upper_diagonal.erase(upper_diagonal.end()-1);
        }


        // rhs elements
        MatrixXd r;
        for (int i = 0; i < number_of_curves - 1; ++i) {
            r = r_matrices[i].transpose() * t_matrices[i].transpose() * data_matrices[i] +
                q_matrices[i+1].transpose() * t_matrices[i+1].transpose() * data_matrices[i+1];
            rhs.push_back(r);
        }
        r = r_matrices[number_of_curves-1].transpose() * t_matrices[number_of_curves-1].transpose()
            * data_matrices[number_of_curves-1];
        if(closed_curve){
            r += q_matrices[0].transpose() * t_matrices[0].transpose() * data_matrices[0];
        }
        rhs.push_back(r);


        // solve the tridiagonal system
        vector<MatrixXd> solution;
        if(!closed_curve){
            solution = solve_tridiagonal(lower_diagonal, diagonal, upper_diagonal, rhs);
        }
        else{
            solution = solve_off_tridiagonal(lower_diagonal, diagonal, upper_diagonal, rhs);
        }


        // extract the control points from the solution
        vector<BezierCurve> bezier_curves;
        vector<VectorXd> control_points;
        VectorXd control_point;
        for(int i = 0; i < solution.size(); i++){
            control_points.clear();

            // if closed curve we need to link last and first solution
            if(i == 0 and closed_curve){
                MatrixXd first_control_points = continuity_matrices[0] * solution[solution.size()-1];
                for(int j = 0; j < first_control_points.rows(); j++){
                    control_point = first_control_points.block(j, 0, 1, first_control_points.cols()).transpose();
                    control_points.push_back(control_point);
                }
            }
            else if(i != 0){
                // if closed curve we have an additional continuity matrix
                MatrixXd continuity_matrix = !closed_curve ? continuity_matrices[i-1] : continuity_matrices[i];
                MatrixXd first_control_points = continuity_matrix * solution[i-1];
                for(int j = 0; j < first_control_points.rows(); j++){
                    control_point = first_control_points.block(j, 0, 1, first_control_points.cols()).transpose();
                    control_points.push_back(control_point);
                }
            }
            for(int j = 0; j < solution[i].rows(); j++){
                control_point = solution[i].block(j, 0, 1, solution[i].cols()).transpose();
                control_points.push_back(control_point);
            }
            bezier_curves.push_back(BezierCurve(control_points));
        }

        return CompositeBezierCurve(bezier_curves);
    }
}

