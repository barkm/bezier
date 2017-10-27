#ifndef BEZIER_BEZIER_POSTSCRIPT_H
#define BEZIER_BEZIER_POSTSCRIPT_H

#include <vector>
#include <array>
#include <string>

#include <bezier/curve.h>
#include <bezier/bezier_curve.h>

#include <fstream>

namespace bezier {
    using Eigen::Vector2d;

    void write_postscript(const std::string & file_name,
                          const std::vector<Curve*> & curves,
                          bool show_control_points);

    void write_postscript(const std::string & file_name,
                          const std::vector<Curve*> & curves,
                          bool show_control_points,
                          const std::array<Vector2d, 2> & limits);

    void _write_control_points(std::ofstream & file, const Vector2d & scale_factor, const vector<VectorXd> & points);

    void _write_bezier_curve(std::ofstream & file, bool show_control_points, const Vector2d & scale_factor, const BezierCurve & bezier);

    void _write_line(std::ofstream & file, const Vector2d & scale_factor, const BezierCurve & line);

    void _write_cubic(std::ofstream & file, const Vector2d & scale_factor, const BezierCurve & cubic);

    void _write_with_samples(std::ofstream & file, const Vector2d & scale_factor, const Curve * curve, int n_samples);
}

#endif //BEZIER_BEZIER_POSTSCRIPT_H
