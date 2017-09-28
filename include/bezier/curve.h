#ifndef BEZIER_CURVE_H
#define BEZIER_CURVE_H

#include <Eigen/Dense>

namespace bezier {

    class Curve {
    public:
        virtual Eigen::Matrix<double, Eigen::Dynamic, 1> operator()(double t) const = 0;
        virtual int degree() const = 0;
        virtual int dimension() const = 0;
    };

}

#endif //BEZIER_CURVE_H
