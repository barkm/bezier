#ifndef BEZIER_CURVE_H
#define BEZIER_CURVE_H

#include <Eigen/Dense>
#include <array>
#include <vector>

namespace bezier {

    class Curve {
    public:
        virtual Eigen::Matrix<double, Eigen::Dynamic, 1> operator()(double t) const = 0;
        virtual unsigned int dimension() const = 0;
        virtual std::array<Eigen::VectorXd, 2> bounds() const = 0;
    };

    std::vector<Eigen::VectorXd> sample(const Curve * curve, int n);
}

#endif //BEZIER_CURVE_H
