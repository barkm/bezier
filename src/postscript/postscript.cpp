#include <bezier/postscript/postscript.h>

#include <iostream>
#include <fstream>

#include <bezier/curve.h>
#include <bezier/bezier_curve.h>
#include <bezier/composite_bezier_curve.h>

using std::vector;
using std::array;
using Eigen::Vector2d;

namespace bezier {
    static const int n_samples = 50;
    static const int line_width = 3;
    static const int circle_radius = 4;
    static const int control_point_line_width = 1;
    static const Vector2d postscript_dims(300, 300);

    void write_postscript(const std::string & file_name,
                          const std::vector<Curve*> & curves,
                          bool show_control_points){
        Vector2d min, max;
        for(Curve * curve : curves){
            std::array<Eigen::VectorXd, 2> bounds = curve->bounds();
            min = bounds[0].cwiseMin(min);
            max = bounds[1].cwiseMax(max);
        }

        write_postscript(file_name, curves, show_control_points, {min, max});
    }

    void write_postscript(const std::string & file_name,
                          const vector<Curve*> & curves,
                          bool show_control_points,
                          const std::array<Vector2d, 2> & limits){

        // assert that all curves are in 2D
        for(Curve * curve : curves){
            if(curve->dimension() != 2){
                throw std::invalid_argument("Curves must be in 2D when writing to PostScript!");
            }
        }

        Vector2d min, max;
        min = limits[0];
        max = limits[1];

        std::ofstream file;
        file.open(file_name);

        if(!file.is_open()){
            throw std::invalid_argument("Could not open file" + file_name);
        }
        
        Vector2d scale_factor = (postscript_dims).cwiseQuotient(max - min);
        min = min.cwiseProduct(scale_factor);
        max = max.cwiseProduct(scale_factor);

        // write post script header
        file << "%!PS-Adobe-3.0 EPSF-3.0\n";
        file << "%%BoundingBox: " << min(0) << " " << min(1) << " " << max(0) << " " << max(1) <<"\n";

        // write all curves
        for(Curve * c : curves){
            auto bezier = dynamic_cast<BezierCurve*>(c);
            if(bezier != nullptr){
                _write_bezier_curve(file, show_control_points, scale_factor, *bezier);
                continue;
            }
            auto composite_bezier = dynamic_cast<CompositeBezierCurve*>(c);
            if(composite_bezier != nullptr){
                for(const BezierCurve & b : composite_bezier->bezier_curves()){
                    _write_bezier_curve(file, show_control_points, scale_factor, b);
                }
                continue;
            }
            throw std::runtime_error("Curve is not BezierCurve nor CompositeBezierCurve!");
        }
    }

    void _write_bezier_curve(std::ofstream & file, bool show_control_points, const Vector2d & scale_factor, const BezierCurve & bezier){
        if(bezier.degree() == 1){
            _write_line(file, scale_factor, bezier);
        }
        else if(bezier.degree() == 3){
            _write_cubic(file, scale_factor, bezier);
        }
        else
        {
            _write_with_samples(file, scale_factor, &bezier, n_samples);
        }

        if(show_control_points){
            _write_control_points(file, scale_factor, bezier.control_points());
        }
    }

    void _write_control_points(std::ofstream & file, const Vector2d & scale_factor, const vector<VectorXd> & points){
        file << "newpath\n";
        Vector2d prev_point;
        for(int i = 0; i < points.size(); i++){
            Vector2d sp = points[i].cwiseProduct(scale_factor);
            file << sp(0) << " " << sp(1) << " " << circle_radius << " 0 360 arc fill\n";
            if (i != 0){
                file << "newpath\n";
                file << prev_point(0) << " " <<
                     prev_point(1) << " moveto " << sp(0) << " " << sp(1) << " lineto\n";
                file << control_point_line_width << " setlinewidth [3 3] 0 setdash\n";
                file << "stroke\n";
                file << "[] 0 setdash\n";
            }
            prev_point = sp;
        }
    }

    void _write_with_samples(std::ofstream & file, const Vector2d & scale_factor, const Curve * curve, int n_samples){
        std::vector<VectorXd> points = sample(curve, n_samples);
        Vector2d p = points[0].cwiseProduct(scale_factor);
        file << "newpath\n" << p(0) << " " << p(1) << " moveto\n";
        for (int i = 1; i < points.size(); ++i) {
            p = points[i].cwiseProduct(scale_factor);
            file << p(0) << " " << p(1) << " lineto\n";
        }
        file << line_width << " setlinewidth\n";
        file << "stroke\n";
    }

    void _write_line(std::ofstream & file, const Vector2d & scale_factor, const BezierCurve & line){
        if(line.degree() != 1){
            throw std::invalid_argument("Not a line!");
        }
        std::vector<VectorXd> control_points = line.control_points();

        Vector2d start = control_points[0].cwiseProduct(scale_factor);
        Vector2d stop = control_points[1].cwiseProduct(scale_factor);

        file << "newpath\n" << start(0) << " " << start(1) << " moveto " << stop(0) << " " << stop(1) << " lineto\n";
        file << line_width << " setlinewidth\n";
        file << "stroke\n";
    }

    void _write_cubic(std::ofstream & file, const Vector2d & scale_factor, const BezierCurve & cubic){
        if(cubic.degree() != 3){
            throw std::invalid_argument("Not a cubic Bezier !");
        }
        std::vector<VectorXd> control_points = cubic.control_points();

        Vector2d c1 = control_points[0].cwiseProduct(scale_factor);
        Vector2d c2 = control_points[1].cwiseProduct(scale_factor);
        Vector2d c3 = control_points[2].cwiseProduct(scale_factor);
        Vector2d c4 = control_points[3].cwiseProduct(scale_factor);

        file << "newpath\n" << c1(0) << " " << c1(1) << " moveto ";
        file << c2(0) << " " << c2(1) << " " << c3(0) << " " << c3(1) << " " <<
             c4(0) << " " << c4(1) << " curveto\n";
        file << line_width << " setlinewidth\n";
        file << "stroke\n";
    }
}
