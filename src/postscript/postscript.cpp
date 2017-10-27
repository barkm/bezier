#include <bezier/postscript/postscript.h>

#include <sstream>

namespace bezier {

    using std::stringstream;

    string ps_circle(const Vector2d & pos, double radius, bool fill, double line_width){
        stringstream circle;
        circle << line_width << " setlinewidth " << pos(0) << " " << pos(1) << " " << radius << " 0 360 arc fill\n";
        return circle.str();
    }

    string ps_line(const Vector2d & start, const Vector2d & stop, double line_width, bool dashed){
        stringstream line;
        line << line_width << " setlinewidth ";
        if(dashed){
            line << "[3 3] 0 setdash ";
        }
        line << start(0) << " " << start(1) << " moveto " <<
             stop(0) << " " << stop(1) << " lineto stroke\n";
        if(dashed){
            line << "[] 0 setdash\n";
        }
        return line.str();
    }

    string ps_polyline(const vector<Vector2d> points, double line_width, bool dashed){
        if (points.size() < 2){
            return "";
        }

        stringstream polyline;
        polyline << line_width << " setlinewidth ";
        if(dashed){
            polyline << "[3 3] 0 setdash ";
        }

        polyline << points[0](0) << " " << points[0](1) << " moveto ";
        for(int i = 1; i < points.size(); i++){
            polyline << points[i](0) << " " << points[i](1) << " lineto ";
        }
        polyline << "stroke\n";
        if(dashed){
            polyline << "[] 0 setdash\n";
        }
        return polyline.str();
    }

    string ps_cubic(const array<Vector2d, 4> control_points, double line_width){
        stringstream cubic;
        cubic << line_width << " setlinewidth ";
        cubic << control_points[0](0) << " " << control_points[0](1) << " moveto ";
        cubic <<
              control_points[1](0) << " " << control_points[1](1) << " " <<
              control_points[2](0) << " " << control_points[2](1) << " " <<
              control_points[3](0) << " " << control_points[3](1) << " curveto stroke\n";
        return cubic.str();
    }

    string ps_header(const array<Vector2d, 2> & limits){
        stringstream header;
        header << "%!PS-Adobe-3.0 EPSF-3.0\n%%BoundingBox: ";
        header << limits[0](0) << " " << limits[0](1) << " " <<
               limits[1](0) << " " << limits[1](1) << "\n";
        return header.str();
    }


}