#ifndef BEZIER_BEZIER_CURVE_H
#define BEZIER_BEZIER_CURVE_H

#include <vector>
#include <Eigen/Dense>

#include <bezier/curve.h>
#include <bezier/math/misc.h>



namespace bezier {
    using std::vector;
    using Eigen::VectorXd;
    using Eigen::MatrixXd;

    Eigen::MatrixXd bezier_coefficients(int degree);

    class BezierCurve : public Curve {
    public:
        explicit BezierCurve(const vector<VectorXd> & control_points);
        BezierCurve(const std::initializer_list<VectorXd> & control_points);

        vector<VectorXd> control_points() const;
        MatrixXd coefficient_matrix() const;

        virtual unsigned int degree() const;
        virtual unsigned int dimension() const;
        virtual VectorXd operator()(double t) const;
        MatrixXd _control_matrix;
    private:
        unsigned int _degree;
        unsigned int _dimension;
        vector<VectorXd> _control_points;
        MatrixXd _coefficient_matrix;
    };
}




#endif //BEZIER_BEZIER_CURVE_H
