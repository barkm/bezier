#include <bezier/curve.h>

namespace bezier {

    std::vector<Eigen::VectorXd> sample(const Curve * curve, int n){

        Eigen::VectorXd params = Eigen::VectorXd::LinSpaced(n, 0, 1);
        std::vector<Eigen::VectorXd> samples;

        for(int i = 0; i < params.rows(); i++){
            samples.push_back(curve->operator()(params(i)));
        }

        return samples;
    }

}
