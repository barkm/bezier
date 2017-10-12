#ifndef BEZIER_COMPOSITE_BEZIER_CURVE_H
#define BEZIER_COMPOSITE_BEZIER_CURVE_H

#include <vector>

#include <bezier/curve.h>
#include <bezier/bezier_curve.h>

namespace bezier {

    using std::vector;
    using Eigen::VectorXd;

    class CompositeBezierCurve : public Curve{
    public:
        explicit CompositeBezierCurve(const vector<vector<VectorXd>>& control_points);
        explicit CompositeBezierCurve(const vector<BezierCurve> & bezier_curves);
        CompositeBezierCurve(const std::initializer_list<BezierCurve> &bezier_curves);
        VectorXd operator()(double t) const;
        virtual unsigned int dimension() const;
        vector<BezierCurve> bezier_curves() const;
    private:
        unsigned int _dimension;
        vector<BezierCurve> _bezier_curves;
    };

    std::pair<int, double> global_to_local_param(double t, int number_of_curves);

}



#endif //BEZIER_COMPOSITE_BEZIER_CURVE_H
