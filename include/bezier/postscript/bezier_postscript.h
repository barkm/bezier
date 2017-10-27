#ifndef BEZIER_BEZIER_POSTSCRIPT_H
#define BEZIER_BEZIER_POSTSCRIPT_H

#include <vector>
#include <array>
#include <string>

#include <bezier/curve.h>
#include <bezier/bezier_curve.h>

#include <bezier/postscript/postscript.h>

#include <fstream>

namespace bezier {
    using Eigen::Vector2d;

    void write_postscript(PostScriptWriter & ps_writer,
                          const std::vector<Curve*> & curves,
                          bool show_control_points);

    void _write_control_points(PostScriptWriter & ps_writer, const vector<VectorXd> & points);

    void _write_bezier_curve(PostScriptWriter & ps_writer, bool show_control_points, const BezierCurve & bezier);

    void _write_line(PostScriptWriter & ps_writer, const BezierCurve & line);

    void _write_cubic(PostScriptWriter & ps_writer, const BezierCurve & cubic);

    void _write_with_samples(PostScriptWriter & ps_writer, const Curve * curve, int n_samples);
}

#endif //BEZIER_BEZIER_POSTSCRIPT_H
