/**
 * Fit composite Bezier curves to noisy circle data.
 * The composite curves consists of Bezier curves of different
 * degrees.
 */

#include <bezier/bezier.h>

using Eigen::Vector2d;
using Eigen::VectorXd;

int main(){

    // Generate points on circle
    Eigen::ArrayXd theta = Eigen::ArrayXd::LinSpaced(30, 0, (1 - 1.0/30.0) * 2 * M_PI);
    Eigen::ArrayXd sin = Eigen::sin(theta);
    Eigen::ArrayXd cos = Eigen::cos(theta);
    Eigen::ArrayXd sin_noise = Eigen::ArrayXd::Random(theta.rows());
    Eigen::ArrayXd cos_noise = Eigen::ArrayXd::Random(theta.rows());
    sin += 0.1 * sin_noise;
    cos += 0.1 * cos_noise;

    // Generate n_circles next to each other
    int n_circles = 4;
    std::vector<std::vector<VectorXd>> circles;
    for(int i = 0; i < n_circles; ++i){
        circles.emplace_back(std::vector<VectorXd>());
        for (int j = 0; j < theta.rows(); ++j) {
            circles[i].emplace_back(Vector2d(cos(j) + 3 * i, sin(j)));
        }
    }

    // Fit composite bezier curves consisting of two bezier curves
    // of varying degree to each circle
    std::vector<bezier::CompositeBezierCurve> beziers;
    for(int i = 0; i <n_circles; ++i){
        bezier::CompositeBezierCurve bezier =
                bezier::fit_composite_bezier_curve(circles[i], {14}, i + 3, true);
        beziers.push_back(bezier);
    }


    // Set up postscript writer
    double x_min = -1.5;
    double x_max = (n_circles-1) * 3 + 1.5;
    double y_min = -2;
    double y_max = 2;
    double ps_y = 300;
    double ps_x = ps_y * (x_max-x_min) / (y_max-y_min);
    bezier::PostScriptWriter ps_writer("circles.eps",
                                       {Vector2d(x_min, y_min), Vector2d(x_max, y_max)},
                                       Vector2d(ps_x, ps_y));

    // Write data points
    ps_writer.filled(true);
    ps_writer.color(0, 0, 1);
    for(const auto & circle : circles){
        for(const auto & p : circle){
            ps_writer.circle(p, 3);
        }
    }
    ps_writer.filled(false);


    // Write bezier curves
    ps_writer.color(0, 0, 0);
    for(int i = 0; i < beziers.size(); ++i){
        bezier::write_curve(ps_writer, &beziers[i], true);
    }

}