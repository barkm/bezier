#ifndef BEZIER_FIT_COMPOSITE_BEZIER_CURVE_H
#define BEZIER_FIT_COMPOSITE_BEZIER_CURVE_H

#include <bezier/composite_bezier_curve.h>


namespace bezier {

    using std::vector;

    CompositeBezierCurve fit_composite_bezier_curve(const vector<vector<VectorXd>> & data_points,
                                                    const vector<vector<double>> & parameterization,
                                                    const vector<int> & curve_degrees,
                                                    bool closed_curve);
}




#endif //BEZIER_FIT_COMPOSITE_BEZIER_CURVE_H
