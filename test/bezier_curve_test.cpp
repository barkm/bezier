#include <catch/catch.hpp>


#include <bezier/bezier_curve.h>

using std::vector;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;


TEST_CASE("Bezier coefficients", "[coefficients]"){
    Eigen::Matrix4d cubic_coeff;
    cubic_coeff <<   1,  0,  0,  0,
                    -3,  3,  0,  0,
                    3, -6,  3,  0,
                    -1,  3, -3,  1;
    REQUIRE(bezier::bezier_coefficients(3) == cubic_coeff);
}

TEST_CASE("Bezier curve construction", "[construction]"){

    // create 3-dimensional linear
    vector<VectorXd> control_points1 = { Vector3d(1, -2, 3), Vector3d(-1, 3, 5) };
    bezier::BezierCurve linear3d(control_points1);

    // create 2-dimensional cubic
    bezier::BezierCurve cubic2d({ Vector2d(1, -1), Vector2d(-1, 3), Vector2d(4, 10), Vector2d(100, -12) });

    // create 5-dimensional constant with initializer list
    bezier::BezierCurve constant4d = {Vector4d(1, 2, 3, 4)};

    SECTION("invalid construction"){
        REQUIRE_THROWS_AS(bezier::BezierCurve({Vector4d(1, 2, 3, 4), Vector3d(1, 2, 3)}), std::invalid_argument);
    }

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
        REQUIRE((linear3d.control_points() == control_points1));
        REQUIRE((cubic2d.control_points() == vector<VectorXd>{Vector2d(1, -1), Vector2d(-1, 3), Vector2d(4, 10), Vector2d(100, -12)}));
        REQUIRE((constant4d.control_points() == vector<VectorXd>{Vector4d(1, 2, 3, 4)}));
    }

    SECTION("coefficient matrix"){
        REQUIRE(linear3d.coefficient_matrix() == bezier::bezier_coefficients(1));
        REQUIRE(cubic2d.coefficient_matrix() == bezier::bezier_coefficients(3));
        REQUIRE(constant4d.coefficient_matrix() == bezier::bezier_coefficients(0));
    }
}


TEST_CASE("Bezier curve evaluation", "[evaluation]"){

    vector<VectorXd> control_points = { Vector2d(1, -1), Vector2d(1, 2), Vector2d(-2, 1), Vector2d(-2, -1) };
    bezier::BezierCurve cubic2d(control_points);

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

    SECTION("curve inheritance"){
        const bezier::Curve & curve = cubic2d;
        REQUIRE(curve(0.5) == Vector2d(-4, 7)/8);
        REQUIRE(curve(0.25) == Vector2d(34, 35)/64);
    }
}

TEST_CASE("Bezier curve bounds", "[bounds]"){
    vector<VectorXd> control_points = { Vector2d(1, -1), Vector2d(1, 2), Vector2d(-2, 1), Vector2d(-2, -1) };
    bezier::BezierCurve cubic2d(control_points);
    std::array<VectorXd, 2> bounds = cubic2d.bounds();
    REQUIRE(bounds[0] == Vector2d(-2, -1));
    REQUIRE(bounds[1] == Vector2d(1, 2));
}



