#include <bezier/composite_bezier_curve.h>
#include <iostream>

namespace bezier {

    CompositeBezierCurve::CompositeBezierCurve(const vector<vector<VectorXd>> & control_points) {
        if(control_points.empty()){
            throw std::invalid_argument("A composite Bezier curve must have at least one Bezier curve.");
        }

        for(const auto & cp : control_points){
            _bezier_curves.emplace_back(cp);
        }

        _dimension = _bezier_curves[0].dimension();

        for(int i = 1; i < _bezier_curves.size(); i++){
            if(_bezier_curves[i-1](0) != _bezier_curves[i](0)){
                throw std::invalid_argument("A composite Bezier curve must be continuous.");
            }
            if(_bezier_curves[i].dimension() != _dimension){
                throw std::invalid_argument("All Bezier curves in a composite Bezier"
                                                    " curve must have the same dimensionality");
            }
        }
    }

    CompositeBezierCurve::CompositeBezierCurve(const vector<BezierCurve> & bezier_curves) {
        if(bezier_curves.empty()){
            throw std::invalid_argument("A composite Bezier curve must have at least one Bezier curve.");
        }

        _dimension = bezier_curves[0].dimension();

        for(int i = 1; i < bezier_curves.size(); i++){
            if(bezier_curves[i-1](1) != bezier_curves[i](0)){
                throw std::invalid_argument("A composite Bezier curve must be continuous.");
            }
            if(bezier_curves[i].dimension() != _dimension){
                throw std::invalid_argument("All Bezier curves in a composite Bezier"
                                                    " curve must have the same dimensionality");
            }
        }

        for(auto it = bezier_curves.begin(); it != bezier_curves.end(); ++it) {
            _bezier_curves.push_back(*it);
        }
    }

    CompositeBezierCurve::CompositeBezierCurve(const std::initializer_list<BezierCurve> &bezier_curves) :
            CompositeBezierCurve(vector<BezierCurve>(bezier_curves)){}

    unsigned int CompositeBezierCurve::dimension() const {
        return _dimension;
    }


    VectorXd CompositeBezierCurve::operator()(double t) const {
        if(t < 0 or t > 1){
            throw std::domain_error("Composite Bezier curve only defined on [0,1].");
        }

        unsigned long curve_index;
        double curve_fraction;
        if (t == 1){
            curve_index = _bezier_curves.size() - 1;
            curve_fraction = 0;
        }
        else{
            curve_index = static_cast<unsigned long>(std::floor(_bezier_curves.size() * t));
            curve_fraction = curve_index / (double) _bezier_curves.size();
        }
        return _bezier_curves[curve_index](t - curve_fraction);
    }
}