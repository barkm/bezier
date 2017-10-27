#ifndef BEZIER_POSTSCRIPT_H
#define BEZIER_POSTSCRIPT_H

#include <string>
#include <array>
#include <vector>

#include <Eigen/Dense>

namespace bezier {
    using std::array;
    using std::vector;
    using Eigen::Vector2d;
    using std::string;

    string ps_circle(const Vector2d & pos, double radius, bool fill, double line_width=1);

    string ps_line(const Vector2d & start, const Vector2d & stop, double line_width=1, bool dashed=false);

    string ps_polyline(const vector<Vector2d> points, double line_width=1, bool dashed=false);

    string ps_cubic(const array<Vector2d, 4> control_points, double line_width = 1);

    string ps_header(const array<Vector2d, 2> & limits);

}

#endif //BEZIER_POSTSCRIPT_H
