#ifndef BEZIER_BEZIER_CURVE_H
#define BEZIER_BEZIER_CURVE_H

#include <array>
#include <Eigen/Dense>

#include <bezier/utilities.h>

namespace bezier {
    using std::array;

    // forward declaration
    template <int deg>
    Eigen::Matrix<double, deg+1, deg+1> bezier_coefficients();

    template <int deg, int dim>
    class BezierCurve {
    public:
        explicit BezierCurve(const array<array<double, dim>, deg+1> & control_points);
        explicit BezierCurve(const array<Eigen::Matrix<double, dim, 1>, deg+1> & control_points);
        array<Eigen::Matrix<double, dim, 1>, deg+1> control_points() const;
        int degree() const;
        int dimension() const;
        Eigen::Matrix<double, deg+1, deg+1> coefficient_matrix();
        Eigen::Matrix<double, dim, 1> operator()(double t) const;
    private:
        array<Eigen::Matrix<double, dim, 1>, deg+1> _control_points;
        Eigen::Matrix<double, deg+1, deg+1> _coefficient_matrix;
        Eigen::Matrix<double, deg+1, dim> _control_matrix;
    };

    template <int deg, int dim>
    BezierCurve<deg, dim>::BezierCurve(const array<array<double, dim>, deg + 1> &control_points) {
        // initialize control matrix
        for(int j = 0; j < deg+1; j++){
            for (int i = 0; i < dim; ++i) {
                _control_points[j](i, 0) = control_points[j][i];
                _control_matrix(j, i) = control_points[j][i];
            }
        }
        // initialize coefficient matrix
        _coefficient_matrix = bezier_coefficients<deg>();
    }


    template <int deg, int dim>
    BezierCurve<deg, dim>::BezierCurve(const array<Eigen::Matrix<double, dim, 1>, deg + 1> &control_points) :
            _control_points(control_points){

        // initialize control matrix
        for(int j = 0; j < deg+1; j++){
            for (int i = 0; i < dim; ++i) {
                _control_matrix(j, i) = control_points[j](i, 0);
            }
        }
        // initialize coefficient matrix
        _coefficient_matrix = bezier_coefficients<deg>();
    }


    template <int deg, int dim>
    array<Eigen::Matrix<double, dim, 1>, deg+1> BezierCurve<deg, dim>::control_points() const {
        return _control_points;
    }

    template <int deg, int dim>
    int BezierCurve<deg, dim>::degree() const {
        return deg;
    }

    template <int deg, int dim>
    int BezierCurve<deg, dim>::dimension() const {
        return dim;
    }

    template <int deg, int dim>
    Eigen::Matrix<double, deg+1, deg+1> BezierCurve<deg, dim>::coefficient_matrix() {
        return _coefficient_matrix;
    }

    template <int deg, int dim>
    Eigen::Matrix<double, dim, 1> BezierCurve<deg, dim>::operator()(double t) const {
        if(t < 0 or t > 1){
            throw std::domain_error("Bezier curve only defined on [0,1].");
        }
        Eigen::Matrix<double, deg+1, 1> tvec;
        double t_pow = 1;
        for(int j = 0; j < deg+1; j++){
            tvec(j, 0) = t_pow;
            t_pow *= t;
        }
        return tvec.transpose() * _coefficient_matrix * _control_matrix;
    }


    template <int deg>
    Eigen::Matrix<double, deg+1, deg+1> bezier_coefficients(){
        Eigen::Matrix<double, deg+1, deg+1> coefficient_matrix;

        for(int j = 0; j < deg+1; j++){
            double coeff = factorial(deg) / factorial(deg - j);
            for(int i = 0; i < deg+1; i++){
                if(i <= j){
                    coefficient_matrix(j, i) = coeff * std::pow(-1, i+j) / (factorial(i) * factorial(j - i));
                }
                else{
                    coefficient_matrix(j, i) = 0;
                }
            }
        }

        return coefficient_matrix;
    };

}




#endif //BEZIER_BEZIER_CURVE_H
