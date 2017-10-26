#ifndef BEZIER_POSTSCRIPT_H
#define BEZIER_POSTSCRIPT_H

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
                          const Vector2d & ps_dims);

    void write_postscript(const std::string & file_name,
                          const std::vector<Curve*> & curves,
                          const std::array<Vector2d, 2> & limits,
                          const Vector2d & ps_dims);

    void _write_line(std::ofstream & file, const Vector2d & scale_factor, const BezierCurve & line);

    void _write_cubic(std::ofstream & file, const Vector2d & scale_factor, const BezierCurve & cubic);

    void _write_with_samples(std::ofstream & file, const Vector2d & scale_factor, Curve * curve, int n_samples);
}

#endif //BEZIER_POSTSCRIPT_H
