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

        unsigned int degree() const;
        unsigned int dimension() const override;
        VectorXd operator()(double t) const override;
        std::array<Eigen::VectorXd, 2> bounds() const override;
    private:
        unsigned int _degree;
        unsigned int _dimension;
        vector<VectorXd> _control_points;
        MatrixXd _coefficient_matrix;
        MatrixXd _control_matrix;
    };
}




#endif //BEZIER_BEZIER_CURVE_H
