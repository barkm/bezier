#include <bezier/bezier_curve.h>

namespace bezier {
    using std::vector;
    using Eigen::VectorXd;
    using Eigen::MatrixXd;

    BezierCurve::BezierCurve(const vector<VectorXd> &control_points) {
        if (control_points.empty()){
            throw std::invalid_argument("Must at least provide one control point.");
        }
        _degree = static_cast<unsigned int>(control_points.size() - 1);
        _dimension = static_cast<unsigned int>(control_points[0].rows());

        for(const auto & control_point : control_points){
            if(control_point.rows() != dimension()){
                throw std::invalid_argument("All control points must have the same dimensionality.");
            }
        }
        _control_points = control_points;

        // initialize control matrix
        _control_matrix.resize(_degree + 1, _dimension);
        for(int j = 0; j < _degree+1; j++){
            for (int i = 0; i < _dimension; ++i) {
                _control_matrix(j, i) = _control_points[j](i);
            }
        }
        _coefficient_matrix = bezier_coefficients(_degree);
    }

    BezierCurve::BezierCurve(const std::initializer_list<VectorXd> &control_points) :
            BezierCurve(vector<VectorXd>(control_points.begin(), control_points.end())) {}


    vector<VectorXd> BezierCurve::control_points() const {
        return _control_points;
    }

    unsigned int BezierCurve::degree() const {
        return _degree;
    }

    unsigned int BezierCurve::dimension() const {
        return _dimension;
    }

    MatrixXd BezierCurve::coefficient_matrix() const{
        return _coefficient_matrix;
    }

    VectorXd BezierCurve::operator()(double t) const {
        if(t < 0 or t > 1){
            throw std::domain_error("Bezier curve only defined on [0,1].");
        }
        VectorXd tvec(_degree + 1);
        double t_pow = 1;
        for(int j = 0; j < _degree+1; j++){
            tvec(j) = t_pow;
            t_pow *= t;
        }
        return tvec.transpose() * _coefficient_matrix * _control_matrix;
    }

    std::array<Eigen::VectorXd, 2> BezierCurve::bounds() const {
        VectorXd min_bound = _control_points[0];
        VectorXd max_bound = _control_points[0];
        for (int i = 1; i < _control_points.size(); ++i) {
            min_bound = min_bound.cwiseMin(_control_points[i]);
            max_bound = max_bound.cwiseMax(_control_points[i]);
        }
        return {min_bound, max_bound};
    }


    MatrixXd bezier_coefficients(int degree){
        MatrixXd coefficient_matrix(degree + 1, degree + 1);

        for(int j = 0; j < degree+1; j++){
            double coeff = factorial(degree) / factorial(degree - j);
            for(int i = 0; i < degree+1; i++){
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


