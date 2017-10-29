/**
 * Fit a single cubic Bezier curve to noisy sine wave data.
 */

#include <bezier/bezier.h>

int main(){

    // Generate data
    Eigen::ArrayXd theta = Eigen::ArrayXd::LinSpaced(20, 0, 2 * M_PI);
    Eigen::ArrayXd noise = Eigen::ArrayXd::Random(theta.rows());
    Eigen::ArrayXd sin = Eigen::sin(theta);
    sin += 0.2 * noise;

    // Construct data vector of angles and noisy sine wave
    std::vector<Eigen::VectorXd> wave;
    for (int i = 0; i < theta.rows(); ++i) {
        wave.emplace_back(Eigen::Vector2d(theta(i), sin(i)));
    }

    // Least square fit bezier curve
    // Fit a single bezier curve (no joints) of degree 3 which is not closed
    bezier::CompositeBezierCurve bezier = bezier::fit_composite_bezier_curve(wave,
                                                                      {},
                                                                      3,
                                                                      false);

    // Set up post script writer
    bezier::PostScriptWriter ps_writer("wave.eps",
                                       {Eigen::Vector2d(-0.5, -4), Eigen::Vector2d(2 * M_PI + 0.5, 4)},
                                       Eigen::Vector2d(600, 300));

    // Write data points
    ps_writer.color(1, 0 ,0);
    ps_writer.filled(true);
    for (int i = 0; i < wave.size(); ++i){
        ps_writer.circle(wave[i], 2);
    }
    ps_writer.filled(false);

    // Write the Bezier curve
    ps_writer.color(0, 0, 0);
    bezier::write_curve(ps_writer, &bezier, true);

}
