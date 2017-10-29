#ifndef BEZIER_CURVE_H
#define BEZIER_CURVE_H

#include <Eigen/Dense>
#include <array>
#include <vector>

namespace bezier {

    /**
     * Curve class.
     * Abstract curve class for curves parameterized on [0,1].
     */
    class Curve {
    public:
        /**
         * Evaluate curve
         * @param t : param
         * @return curve value at t
         */
        virtual Eigen::Matrix<double, Eigen::Dynamic, 1> operator()(double t) const = 0;

        /**
         * Dimension of the curve's range space
         * @return d
         */
        virtual unsigned int dimension() const = 0;

        /**
         * Bounds on the curve
         * @return {lower bound, upper bound}
         */
        virtual std::array<Eigen::VectorXd, 2> bounds() const = 0;
    };

    /**
     * Sample uniformuously from a curve in parameter space
     * @param curve : curve to sample from
     * @param n : number of samples
     * @return list of samples
     */
    std::vector<Eigen::VectorXd> sample(const Curve * curve, int n);
}

#endif //BEZIER_CURVE_H
