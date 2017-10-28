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
    /**
     * Least square fit composite Bezier curve
     * @param data_points : data points associated with each curve
     * @param curve_degrees : degree of each curve
     * @param closed_curve : if the fitted curve should be closed
     * @return compsite bezier curve
     */
    CompositeBezierCurve fit_composite_bezier_curve(const vector<vector<VectorXd>> & data_points,
                                                    const vector<int> & curve_degrees,
                                                    bool closed_curve);

    /**
     * Least square fit composite Bezier curve
     * @param data_points : data points
     * @param joints : indices where to split the data points between curves
     * @param curve_degrees : degree of each curve
     * @param closed_curve : if the fitted curve should be closed
     * @return composite bezier curve
     */
    CompositeBezierCurve fit_composite_bezier_curve(const vector<VectorXd> & data_points,
                                                    const vector<int> & joints,
                                                    const vector<int> & curve_degrees,
                                                    bool closed_curve);

    /**
     * Least square fit composite Bezier curve
     * @param data_points : data points
     * @param joints : indices where to split the data points between curves
     * @param curve_degrees : degree of each curve
     * @param closed_curve : if the fitted curve should be closed
     * @return composite bezier curve
     */
    CompositeBezierCurve fit_composite_bezier_curve(const vector<VectorXd> & data_points,
                                                    const vector<int> & joints,
                                                    int curve_degrees,
                                                    bool closed_curve);

    vector<double> chordlength_parameterization(const vector<VectorXd> &data_points,
                                                const VectorXd &start_point = VectorXd(0));

    vector<vector<double>> initialize_parameterization(const vector<vector<VectorXd>> & data_points, bool closed_curve);

    MatrixXd parameterization_matrix(const vector<double> &parameterization, int degree);
}




#endif //BEZIER_FIT_COMPOSITE_BEZIER_CURVE_H
