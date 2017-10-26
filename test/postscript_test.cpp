#define CATCH_CONFIG_MAIN
#include <catch/catch.hpp>

#include <bezier/postscript/postscript.h>

#include <bezier/bezier_curve.h>

using Eigen::Vector2d;

TEST_CASE("Postscript test", "[postscript]"){
    bezier::BezierCurve curve1({ Vector2d(1, -1), Vector2d(-1, 3), Vector2d(4, 10), Vector2d(100, -12) });
    bezier::BezierCurve curve2({ Vector2d(1, -1), Vector2d(1, 2)});
    bezier::BezierCurve curve3({ Vector2d(150, -20), Vector2d(50, 20), Vector2d(40, 30), Vector2d(0, -10),
                               Vector2d(-20, 30)});
    REQUIRE_NOTHROW(bezier::write_postscript("test.ps", {&curve1, &curve2, &curve3}, Vector2d(100, 100)));
}
