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

    /**
     * Bezier curve class.
     * An n-degree Bezier curve is a function B:[0,1]->R^d which defined by n+1 control points in R^d.
     * In detail we have that
     *
     * B(t) = T * C * P
     *
     * where
     *
     * T = (1 t t^2 ... t^n) in R^(1 x n+1),
     *
     * C in R^(n+1 x n+1) is the coefficient matrix,
     *
     * P = (P_0, P_1, P_2, ... , P_n) in R^(n+1 x d)
     *
     * where P_i in R^d is control point i.
     */
    class BezierCurve : public Curve {
    public:
        /**
         * Construct the Bezier curve.
         * The degree and dimension are implictly derived from the arguments.
         * If a list of n+1 control points in R^d is supplied, then an n-degree
         * Bezier curve from [0,1]->R^d will be constructed.
         * @param control_points : control points defining the curve
         */
        explicit BezierCurve(const vector<VectorXd> & control_points);
        BezierCurve(const std::initializer_list<VectorXd> & control_points);

        /**
         * Retrieve the control points
         * @return control points
         */
        vector<VectorXd> control_points() const;

        /**
         * Retrieve the coefficient matrix.
         * @return coefficient matrix in R^(n+1 x n+1)
         */
        MatrixXd coefficient_matrix() const;

        /**
         * Retrieve the degree of the Bezier curve
         * @return n
         */
        unsigned int degree() const;

        /**
         * Retrieve the output dimension of the Bezier curve
         * @return d
         */
        unsigned int dimension() const override;

        /**
         * Evaluate the Bezier curve at t
         * @param t : parameter value
         * @return vector in R^d
         */
        VectorXd operator()(double t) const override;

        /**
         * Estimated bounds of the Bezier curve.
         * Computed from the mininum and maximum values of the control points in each
         * dimension, which are known to contain the curve.
         * @return {lower bound, upper bound)
         */
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
