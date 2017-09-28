#define CATCH_CONFIG_MAIN
#include <catch/catch.hpp>

#include <array>

#include <bezier/bezier_curve.hpp>

using std::array;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;


TEST_CASE("Bezier curve construction", "[construction]"){

    // create 3-dimensional linear
    array<array<double, 3>, 2> control_points1 = {{ {{1, -2, 3}}, {{-1, 3, 5}} }};
    bezier::BezierCurve<1, 3> linear3d(control_points1);

    // create 2-dimensional cubic
    array<Vector2d, 4> control_points2 = {{ Vector2d(1, -1), Vector2d(-1, 3), Vector2d(4, 10), Vector2d(100, -12) }};
    bezier::BezierCurve<3, 2> cubic2d(control_points2);

    // create 5-dimensional constant
    array<Vector4d, 1> control_points3 = {Vector4d(1, 2, 3, 4)};
    bezier::BezierCurve<0, 4> constant4d(control_points3);

    SECTION("degree"){
        REQUIRE(linear3d.degree() == 1);
        REQUIRE(cubic2d.degree() == 3);
        REQUIRE(constant4d.degree() == 0);
    }

    SECTION("dimension"){
        REQUIRE(linear3d.dimension() == 3);
        REQUIRE(cubic2d.dimension() == 2);
        REQUIRE(constant4d.dimension() == 4);
    }

    SECTION("control points") {
        REQUIRE((linear3d.control_points() == array<Vector3d, 2>{{Vector3d(1, -2, 3), Vector3d(-1, 3, 5)}}));
        REQUIRE((cubic2d.control_points() == control_points2));
        REQUIRE((constant4d.control_points() == control_points3));
    }

    SECTION("coefficient matrix"){

        Eigen::Matrix4d cubic_coeff;
        cubic_coeff <<   1,  0,  0,  0,
                        -3,  3,  0,  0,
                         3, -6,  3,  0,
                        -1,  3, -3,  1;
        REQUIRE(cubic2d.coefficient_matrix() == cubic_coeff);
    }
}

TEST_CASE("Bezier curve evaluation", "[evaluation]"){

    array<array<double, 2>, 4> control_points = {{ {{1, -1}}, {{1, 2}}, {{-2, 1}}, {{-2, -1}} }};
    bezier::BezierCurve<3, 2> cubic2d(control_points);

    SECTION("test domain range"){
        REQUIRE_THROWS_AS(cubic2d(-1), std::domain_error);
        REQUIRE_THROWS_AS(cubic2d(2), std::domain_error);
        REQUIRE_NOTHROW(cubic2d(0));
        REQUIRE_NOTHROW(cubic2d(0.5));
        REQUIRE_NOTHROW(cubic2d(1));
    }

    SECTION("end points"){
        REQUIRE(cubic2d(0) == Eigen::Vector2d(1, -1));
        REQUIRE(cubic2d(1) == Eigen::Vector2d(-2, -1));
    }

    SECTION("random points"){
        REQUIRE(cubic2d(0.5) == Vector2d(-4, 7)/8);
        REQUIRE(cubic2d(0.25) == Vector2d(34, 35)/64);
    }
}



