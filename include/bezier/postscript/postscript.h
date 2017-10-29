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

    /**
     * PostScript writer class.
     * Class for generating PostScript files.
     */
    class PostScriptWriter {
    public:
        /**
         * Construct the writer such that the output is written directly to a specified file.
         * @param filename : name of the file
         * @param limits : lower and upper bounds of the data to write
         * @param ps_dims : resolution of the PostScript file
         */
        PostScriptWriter(const string & filename,
                         const array<Vector2d, 2> & limits,
                         const Vector2d & ps_dims=DEFAULT_DIMS);

        /**
         * Construct the writer such that the output is not directly written to file.
         * @param limits : lower and upper bounds of the data to write.
         * @param ps_dims : resolution of the PostScript file
         */
        explicit PostScriptWriter(const array<Vector2d, 2> & limits,
                                  const Vector2d & ps_dims=DEFAULT_DIMS);

        /**
         * Write a circle
         * @param pos : center of circle
         * @param radius : radius of circle
         */
        void circle(const Vector2d & pos, double radius);

        /**
         * Write a number of circles
         * @param pos : center of the circles
         * @param radius : radius of the circles
         */
        void circles(const vector<Vector2d> & pos, double radius);

        /**
         * Write a line
         * @param start : start of line
         * @param stop : end of line
         */
        void line(const Vector2d & start, const Vector2d & stop);

        /**
         * Write a C^0 continuous polyline
         * @param points : start, joints and end point of the polyline
         */
        void polyline(const vector<Vector2d> & points);

        /**
         * Write a cubic Bezier curve
         * @param control_points : control points
         */
        void cubic(const array<Vector2d, 4> & control_points);

        /**
         * Set the color
         * @param r : red value
         * @param g : green value
         * @param b : blue value
         */
        void color(double r, double g, double b);

        /**
         * Set line width
         * @param w : width
         */
        void line_width(double w);

        /**
         * Set dashed. [3 3] 0 setdash is used.
         * @param b : true/false
         */
        void dashed(bool b);

        /**
         * Set filled.
         * @param b : true/false
         */
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

    string ps_header(const array<Vector2d, 2> & limits);
}

#endif //BEZIER_POSTSCRIPT_H
