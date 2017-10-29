#ifndef BEZIER_COMPOSITE_BEZIER_CURVE_H
#define BEZIER_COMPOSITE_BEZIER_CURVE_H

#include <vector>

#include <bezier/curve.h>
#include <bezier/bezier_curve.h>

namespace bezier {

    using std::vector;
    using Eigen::VectorXd;

    /**
     * Composite Bezier curve class.
     * A composite Bezier curve is a piecewise Bezier curve which is at least C^0 continuous.
     * It is here defined as a function B:[0,1]->R^d similarly to the Bezier curve parameterization.
     */
    class CompositeBezierCurve : public Curve{
    public:

        /**
         * Construct the composite Bezier curve.
         * @param control_points : control points defining the Bezier curves.
         */
        explicit CompositeBezierCurve(const vector<vector<VectorXd>>& control_points);

        /**
         * Construct the composite Bezier curve.
         * @param bezier_curves : bezier curves defining the composite Bezier curve.
         */
        explicit CompositeBezierCurve(const vector<BezierCurve> & bezier_curves);
        CompositeBezierCurve(const std::initializer_list<BezierCurve> &bezier_curves);

        /**
         * Evaluate the composite Bezier curve at t.
         * Which of the constituing Bezier curve is evaluated is determined by the number of curves.
         * @param t : parameter
         * @return vector in R^d
         */
        VectorXd operator()(double t) const override;

        /**
         * Retrieve dimension of the curve
         * @return d
         */
        unsigned int dimension() const override;

        /**
         * Bounds of the curve
         * @return {lower bound, upper bound}
         */
        std::array<Eigen::VectorXd, 2> bounds() const override;

        /**
         * Retrieve the Bezier curves.
         * @return list of Bezier curves.
         */
        vector<BezierCurve> bezier_curves() const;
    private:
        unsigned int _dimension;
        vector<BezierCurve> _bezier_curves;
    };

    std::pair<int, double> global_to_local_param(double t, int number_of_curves);

}



#endif //BEZIER_COMPOSITE_BEZIER_CURVE_H
