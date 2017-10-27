#ifndef BEZIER_POSTSCRIPT_H
#define BEZIER_POSTSCRIPT_H

#include <string>
#include <array>
#include <vector>

#include <Eigen/Dense>
#include <fstream>

namespace bezier {
    using std::array;
    using std::vector;
    using Eigen::Vector2d;
    using Eigen::Vector3d;
    using std::string;
    using std::stringstream;
    using std::ofstream;
    using std::ostream;

    const static Vector2d DEFAULT_DIMS(300, 300);

    class PostScriptWriter {
    public:
        PostScriptWriter(const string & filename,
                         const array<Vector2d, 2> & limits,
                         const Vector2d & ps_dims=DEFAULT_DIMS);

        explicit PostScriptWriter(const array<Vector2d, 2> & limits,
                                  const Vector2d & ps_dims=DEFAULT_DIMS);

        void circle(const Vector2d & pos, double radius);
        void circles(const vector<Vector2d> & pos, double radius);
        void line(const Vector2d & start, const Vector2d & stop);
        void polyline(const vector<Vector2d> & points);
        void cubic(const array<Vector2d, 4> & control_points);

        void color(double r, double g, double b);
        void line_width(double w);
        void dashed(bool b);
        void filled(bool b);

        ~PostScriptWriter();

        PostScriptWriter(const PostScriptWriter &) = delete;
        PostScriptWriter(const PostScriptWriter &&) = delete;

        PostScriptWriter operator=(const PostScriptWriter &) = delete;
        PostScriptWriter operator=(const PostScriptWriter &&) = delete;
    private:
        void header(const array<Vector2d, 2> & limits, const Vector2d & ps_dims);
        string draw();

        Vector2d _scale_factors;
        array<Vector2d, 2> _limits;

        ostream & _outputstream;
        stringstream _postscript;
        ofstream _file;

        Vector3d _color = Vector3d(0, 0, 0);
        double _line_width = 1;
        bool _dashed = false;
        bool _fill = false;
    };

    string ps_circle(const Vector2d & pos, double radius, bool fill, double line_width=1);

    string ps_line(const Vector2d & start, const Vector2d & stop, double line_width=1, bool dashed=false);

    string ps_polyline(const vector<Vector2d> points, double line_width=1, bool dashed=false);

    string ps_cubic(const array<Vector2d, 4> control_points, double line_width = 1);

    string ps_header(const array<Vector2d, 2> & limits);

}

#endif //BEZIER_POSTSCRIPT_H
