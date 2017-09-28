#ifndef BEZIER_BEZIER_CURVE_H
#define BEZIER_BEZIER_CURVE_H

#include <vector>
#include <Eigen/Dense>

#include <bezier/curve.h>
#include <bezier/math/misc.h>


namespace bezier {
    using std::vector;

    Eigen::MatrixXd bezier_coefficients(int degree);

    class BezierCurve : public Curve {
    public:
        explicit BezierCurve(const vector<Eigen::VectorXd> & control_points);
        BezierCurve(const std::initializer_list<Eigen::VectorXd> & control_points);

        vector<Eigen::VectorXd> control_points() const;
        Eigen::MatrixXd coefficient_matrix() const;

        virtual int degree() const;
        virtual int dimension() const;
        virtual Eigen::VectorXd operator()(double t) const;
    private:
        unsigned int _degree;
        unsigned int _dimension;
        vector<Eigen::VectorXd> _control_points;
        Eigen::MatrixXd _coefficient_matrix;
        Eigen::MatrixXd _control_matrix;
    };
}




#endif //BEZIER_BEZIER_CURVE_H
