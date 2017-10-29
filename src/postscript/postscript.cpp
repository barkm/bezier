#include <bezier/postscript/postscript.h>

#include <sstream>

namespace bezier {

    using std::stringstream;



    PostScriptWriter::PostScriptWriter(const array<Vector2d, 2> &limits, const Vector2d & ps_dims) :
            _outputstream(_postscript) {
        header(limits, ps_dims);
    }

    PostScriptWriter::PostScriptWriter(const string &filename,
                                       const array<Vector2d, 2> &limits,
                                       const Vector2d & ps_dims) :
            _outputstream(_file) {
        _file.open(filename);
        if(!_file.is_open()){
            throw std::runtime_error("Could not open file" + filename);
        }
        header(limits, ps_dims);
    }

    PostScriptWriter::~PostScriptWriter() {
        if(_file.is_open()){
            _file.close();
        }
    }

    void PostScriptWriter::header(const array<Vector2d, 2> & limits, const Vector2d & ps_dims) {
        _scale_factors = (ps_dims).cwiseQuotient(limits[1] - limits[0]);
        _limits[0] = limits[0].cwiseProduct(_scale_factors);
        _limits[1] = limits[1].cwiseProduct(_scale_factors);
        _outputstream << ps_header(_limits);
    }

    string PostScriptWriter::draw() {
        return _fill ? "fill" : "stroke";
    }

    void PostScriptWriter::circle(const Vector2d & pos, double radius){
        Vector2d scaled_pos = pos.cwiseProduct(_scale_factors);
        _outputstream << scaled_pos(0) << " " << scaled_pos(1) << " " << radius << " 0 360 arc " << draw() << "\n";
    }

    void PostScriptWriter::circles(const vector<Vector2d> & positions, double radius){
        for(const auto & pos : positions){
            Vector2d scaled_pos = pos.cwiseProduct(_scale_factors);
            _outputstream << scaled_pos(0) << " " << scaled_pos(1) << " " << radius << " 0 360 arc " << draw() << "\n";
        }
    }

    void PostScriptWriter::line(const Vector2d & start, const Vector2d & stop){
        Vector2d scaled_start = start.cwiseProduct(_scale_factors);
        Vector2d scaled_stop = stop.cwiseProduct(_scale_factors);
        _outputstream << scaled_start(0) << " " << scaled_start(1) << " moveto " <<
                      scaled_stop(0) << " " << scaled_stop(1) << " lineto " << draw() << "\n";
    }

    void PostScriptWriter::polyline(const vector<Vector2d> & points){
        Vector2d scaled_point = points[0].cwiseProduct(_scale_factors);
        _outputstream << scaled_point(0) << " " << scaled_point(1) << " moveto ";
        for(int i = 1; i < points.size(); i++){
            scaled_point = points[i].cwiseProduct(_scale_factors);
            _outputstream << scaled_point(0) << " " << scaled_point(1) << " lineto ";
        }
        _outputstream << draw() << "\n";
    }

    void PostScriptWriter::cubic(const array<Vector2d, 4> & control_points){
        Vector2d c0 = control_points[0].cwiseProduct(_scale_factors);
        Vector2d c1 = control_points[1].cwiseProduct(_scale_factors);
        Vector2d c2 = control_points[2].cwiseProduct(_scale_factors);
        Vector2d c3 = control_points[3].cwiseProduct(_scale_factors);

        _outputstream << c0(0) << " " << c0(1) << " moveto ";
        _outputstream <<
                      c1(0) << " " << c1(1) << " " <<
                      c2(0) << " " << c2(1) << " " <<
                      c3(0) << " " << c3(1) << " curveto " << draw() << "\n";
    }

    void PostScriptWriter::color(double r, double g, double b){
        Vector3d new_color(r, g, b);
        if(_color != new_color){
            _color = new_color;
            _outputstream << _color(0) << " " << _color(1) << " " << _color(2) << " setrgbcolor\n";
        }
    }

    void PostScriptWriter::line_width(double w){
        if(_line_width != w){
            _line_width = w;
            _outputstream << _line_width << " setlinewidth\n";
        }
    }

    void PostScriptWriter::dashed(bool b){
        if(_dashed != b){
            _dashed = b;
            if(_dashed){
                _outputstream << "[3 3] 0 setdash\n";
            }
            else{
                _outputstream << "[] 0 setdash\n";
            }
        }
    }


    void PostScriptWriter::filled(bool b){
        if(_fill != b){
            _fill = b;
        }
    }

    string ps_header(const array<Vector2d, 2> & limits){
        stringstream header;
        header << "%!PS-Adobe-3.0 EPSF-3.0\n%%BoundingBox: ";
        header << limits[0](0) << " " << limits[0](1) << " " <<
               limits[1](0) << " " << limits[1](1) << "\n";
        return header.str();
    }


}