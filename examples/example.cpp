/**
 * Fit a composite Bezier curve of piecewise Bezier curves of varying degree
 * to sine wave data.
 */

#include <bezier/bezier.h>
using namespace Eigen;
int main(){
  // Generate sin wave data
  ArrayXd theta = ArrayXd::LinSpaced(30, 0, 2 * M_PI);
  ArrayXd sin = Eigen::sin(theta);
  std::vector<VectorXd> data;
  for(int i = 0; i < theta.rows(); ++i){
    data.push_back(Vector2d(theta(i), sin(i)));
  }

  // Fit composite Bezier curve
  bezier::CompositeBezierCurve bezier =
                bezier::fit_composite_bezier_curve(
                                                  data,       // the data
                                                  {9, 19},    // joints
                                                  {3, 4, 5},  // curve degrees
                                                  false       // closed curve
                                                  );

  // Write output to postscript file
  bezier::PostScriptWriter ps_writer(
                                    "example.eps",                                    // file name
                                    {Vector2d(-0.5, -1.5), Vector2d(2*M_PI+0.5, 1.5)} // limits
                                    );
  ps_writer.color(1, 0, 0); ps_writer.filled(true);
  for(const auto & p : data){
    ps_writer.circle(p, 3);
  }
  ps_writer.color(0, 0, 0); ps_writer.filled(false);
  bezier::write_curve(ps_writer, &bezier);
}

