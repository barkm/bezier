#include <catch/catch.hpp>

#include <bezier/curve.h>
#include <bezier/bezier_curve.h>

using Eigen::Vector2d;
using Eigen::VectorXd;

TEST_CASE("Test curve sampling", "[sample]"){
    bezier::BezierCurve curve({ Vector2d(1, -1), Vector2d(-1, 3), Vector2d(4, 10), Vector2d(100, -12) });
    std::vector<VectorXd> samples = bezier::sample(&curve, 100);
    REQUIRE(samples.size() == 100);
}
