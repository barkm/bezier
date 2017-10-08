#ifndef BEZIER_FIT_COMPOSITE_BEZIER_CURVE_H
#define BEZIER_FIT_COMPOSITE_BEZIER_CURVE_H

#include <bezier/composite_bezier_curve.h>


namespace bezier {

    using std::vector;

    /**
     * Least square fits a composite bezier curve to a set of parameterized data points
     * associated with each bezier curve in the composite curve.
     * @param data_points : data points for each curve
     * @param parameterization : parameterization associated with each data point in the data_points parameter
     * for the corresponding curve.
     * @param curve_degrees : degree of each curve.
     * @param closed_curve : if the fitted curve should be closed.
     * @return composite bezier curve
     */
    CompositeBezierCurve fit_composite_bezier_curve(const vector<vector<VectorXd>> & data_points,
                                                    const vector<vector<double>> & parameterization,
                                                    const vector<int> & curve_degrees,
                                                    bool closed_curve);
}




#endif //BEZIER_FIT_COMPOSITE_BEZIER_CURVE_H
