/**
 * Fit composite Bezier curve of piecewise cubic Bezier curves to
 * wave circle data such that the composite curve interpolates each
 * data point.
 */

#include <bezier/bezier.h>

using Eigen::Vector2d;
using Eigen::VectorXd;

int main(){

    // Generate points on circle
    int n_points = 50;
    Eigen::ArrayXd theta = Eigen::ArrayXd::LinSpaced(n_points, 0, (1 - 1.0/n_points) * 2 * M_PI);
    Eigen::ArrayXd sin = Eigen::sin(theta);
    Eigen::ArrayXd cos = Eigen::cos(theta);

    // Generate a circle with wavy contour
    std::vector<Eigen::VectorXd> circle;
    for (int j = 0; j < theta.rows(); ++j) {
        Vector2d circle_point(cos(j), sin(j));
        circle_point = circle_point * (0.1 * sin(5 * j % n_points) + 1);
        circle.push_back(circle_point);
    }

    // Set joints to be equally spaced and choose number of curves such that
    // the data points are interpolated
    int bezier_degree = 3;
    int number_of_curves = static_cast<int>(static_cast<double>(theta.rows()) / (bezier_degree - 1));
    std::vector<int> joints;
    for(int i = 1; i < number_of_curves; i++){
        joints.push_back((bezier_degree-1) * i - 1);
    }

    // Fit bezier curves
    bezier::CompositeBezierCurve bezier = bezier::fit_composite_bezier_curve(circle, joints, bezier_degree, true);

    // Set up postscript writer
    double x_min = -1.5;
    double x_max = 1.5;
    double y_min = -1.5;
    double y_max = 1.5;
    bezier::PostScriptWriter ps_writer("interpolation.eps", {Vector2d(x_min, y_min), Vector2d(x_max, y_max)});

    // Write data points
    ps_writer.filled(true);
    ps_writer.color(0, 0, 1);
    for(const auto & p : circle){
        ps_writer.circle(p, 3);
    }
    ps_writer.filled(false);


    // Write bezier curves
    ps_writer.color(0, 0, 0);
    bezier::write_curve(ps_writer, &bezier, false);

}
