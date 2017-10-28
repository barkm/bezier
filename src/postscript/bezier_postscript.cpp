#include <bezier/postscript/bezier_postscript.h>

#include <fstream>

#include <bezier/curve.h>
#include <bezier/bezier_curve.h>
#include <bezier/composite_bezier_curve.h>

#include <bezier/postscript/postscript.h>

using std::vector;
using std::array;
using Eigen::Vector2d;

namespace bezier {
    static const int n_samples = 50;
    static const int line_width = 3;
    static const int circle_radius = 4;
    static const int control_point_line_width = 1;
    static const Vector2d postscript_dims(300, 300);

    void write_curve(PostScriptWriter &ps_writer,
                     const Curve *curve,
                     bool show_control_points){

        // assert the curves are in 2D
        if(curve->dimension() != 2){
            throw std::invalid_argument("Curves must be in 2D when writing to PostScript!");
        }

        // write curve
        auto bezier = dynamic_cast<const BezierCurve*>(curve);
        if(bezier != nullptr){
            _write_bezier_curve(ps_writer, show_control_points, *bezier);
        }

        auto composite_bezier = dynamic_cast<const CompositeBezierCurve*>(curve);
        if(composite_bezier != nullptr){
            for(const BezierCurve & b : composite_bezier->bezier_curves()){
                _write_bezier_curve(ps_writer, show_control_points, b);
            }
        }
    }

    void _write_bezier_curve(PostScriptWriter & ps_writer, bool show_control_points, const BezierCurve & bezier){
        if(bezier.degree() == 1){
            _write_line(ps_writer, bezier);
        }
        else if(bezier.degree() == 3){
            _write_cubic(ps_writer, bezier);
        }
        else {
            _write_with_samples(ps_writer, &bezier, n_samples);
        }

        if(show_control_points){
            _write_control_points(ps_writer, bezier.control_points());
        }
    }

    void _write_control_points(PostScriptWriter & ps_writer, const vector<VectorXd> & points){
        vector<Vector2d> p(points.size());
        std::copy(points.begin(), points.end(), p.begin());
        ps_writer.filled(true);
        ps_writer.circles(p, circle_radius);
        ps_writer.filled(false);
        ps_writer.dashed(true);
        ps_writer.polyline(p);
        ps_writer.dashed(false);
    }

    void _write_with_samples(PostScriptWriter & ps_writer, const Curve * curve, int n_samples){
        std::vector<VectorXd> points = sample(curve, n_samples);
        vector<Vector2d> p(points.size());
        std::copy(points.begin(), points.end(), p.begin());
        ps_writer.polyline(p);
    }

    void _write_line(PostScriptWriter & ps_writer, const BezierCurve & line){
        if(line.degree() != 1){
            throw std::invalid_argument("Not a line!");
        }
        ps_writer.line(line.control_points()[0], line.control_points()[1]);
    }

    void _write_cubic(PostScriptWriter & ps_writer, const BezierCurve & cubic){
        if(cubic.degree() != 3){
            throw std::invalid_argument("Not a cubic Bezier !");
        }
        array<Vector2d, 4> p;
        p[0] = cubic.control_points()[0];
        p[1] = cubic.control_points()[1];
        p[2] = cubic.control_points()[2];
        p[3] = cubic.control_points()[3];

        ps_writer.cubic(p);
    }
}
