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

    /**
     * Write curve to PostScript
     * @param ps_writer : PostScriptWriter to write to
     * @param curve : composite Bezier curve or Bezier curve
     * @param show_control_points : true/false
     */
    void write_curve(PostScriptWriter &ps_writer,
                     const Curve *curve,
                     bool show_control_points=false);

    void _write_control_points(PostScriptWriter & ps_writer, const vector<VectorXd> & points);

    void _write_bezier_curve(PostScriptWriter & ps_writer, bool show_control_points, const BezierCurve & bezier);

    void _write_line(PostScriptWriter & ps_writer, const BezierCurve & line);

    void _write_cubic(PostScriptWriter & ps_writer, const BezierCurve & cubic);

    void _write_with_samples(PostScriptWriter & ps_writer, const Curve * curve, int n_samples);
}

#endif //BEZIER_BEZIER_POSTSCRIPT_H
