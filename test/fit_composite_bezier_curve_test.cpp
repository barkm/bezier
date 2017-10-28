#include <catch/catch.hpp>

#include <bezier/fit_composite_bezier_curve.h>

using std::vector;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;
typedef Eigen::Matrix<double, 1, 1> Scalar;


TEST_CASE("Chordlength parameterization tests", "[chordlengthparam]"){

    vector<VectorXd> data_points = {Vector2d(0, 1), Vector2d(0, 3), Vector2d(0, 4)};
    Vector2d start_point(0, 0);

    vector<double> parameterization = bezier::chordlength_parameterization(data_points, start_point);

    REQUIRE(parameterization[0] == 0.25);
    REQUIRE(parameterization[1] == 0.75);
    REQUIRE(parameterization[2] == 1);
}

TEST_CASE("Initializing parameterization test", "[initparam]"){
    vector<vector<VectorXd>> data_points = {{Vector2d(0, 0), Vector2d(0, 2.5), Vector2d(0, 5)},
                                    {Vector2d(0, 6), Vector2d(0, 10), Vector2d(0, 15), Vector2d(0, 25)}};
    vector<vector<double>> parameterization = bezier::initialize_parameterization(data_points, false);

    REQUIRE(parameterization[0][0] == 0);
    REQUIRE(parameterization[0][1] == 0.5);
    REQUIRE(parameterization[0][2] == 1);

    REQUIRE(parameterization[1][0] == 0.05);
    REQUIRE(parameterization[1][1] == 0.25);
    REQUIRE(parameterization[1][2] == 0.5);
    REQUIRE(parameterization[1][3] == 1);
}


TEST_CASE("Fit with parameterization tests", "[with parameterization]"){

    SECTION("invalid arguments"){
        SECTION("too low degree curve"){
            vector<vector<VectorXd>> data_points = {{Vector3d(2, 1, 0), Vector3d(-1, 3, 2), Vector3d(10, 2, -1), Vector3d(5, 3, 2)},
                           {Vector3d(5, 3, 2), Vector3d(1, 0, 0)},
                           {Vector3d(1, 0, 0), Vector3d(5, 4, 3)}};
            vector<vector<double>> parameterization = {{0, 0.25, 0.75, 1}, {0.25, 1}, {0.25, 0.75, 1}};
            vector<int> curve_degrees = {3, 3, 2};
            bool closed_curve = false;

            REQUIRE_THROWS_AS(bezier::fit_composite_bezier_curve(data_points,
                                                                 parameterization,
                                                                 curve_degrees,
                                                                 closed_curve),
                              std::invalid_argument);
        }

        SECTION("too few data points for curve"){
            vector<vector<VectorXd>> data_points = {{Vector2d(2, 1), Vector2d(-1, 3), Vector2d(10, 2), Vector2d(5, 3)}};
            vector<vector<double>> parameterization = {{0, 0.25, 0.75, 1}};
            vector<int> curve_degrees = {4};
            bool closed_curve = false;

            REQUIRE_THROWS_AS(bezier::fit_composite_bezier_curve(data_points,
                                                                 parameterization,
                                                                 curve_degrees,
                                                                 closed_curve),
                              std::invalid_argument);

            data_points = {{Vector3d(2, 1, 0), Vector3d(-1, 3, 2), Vector3d(10, 2, -1), Vector3d(5, 3, 2)},
                           {Vector3d(5, 3, 2), Vector3d(1, 0, 0)},
                           {Vector3d(1, 0, 0), Vector3d(5, 4, 3)}};
            parameterization = {{0, 0.25, 0.75, 1}, {0.25, 1}, {0.25, 0.75, 1}};
            curve_degrees = {3, 3, 4};
            closed_curve = false;

            REQUIRE_THROWS_AS(bezier::fit_composite_bezier_curve(data_points,
                                                                 parameterization,
                                                                 curve_degrees,
                                                                 closed_curve),
                              std::invalid_argument);
        }

        SECTION("parameterization not given for every point"){
            vector<vector<VectorXd>>data_points = {{Scalar(0.232), Scalar(0.43), Scalar(0.3434),
                                                           Scalar(0.343), Scalar(0.123), Scalar(-0.32)}};
            vector<vector<double>> parameterization = {{0, 0.1, 0.4, 0.5, 0.75}};
            vector<int> curve_degrees = {5};
            bool closed_curve = false;

            REQUIRE_THROWS_AS(bezier::fit_composite_bezier_curve(data_points,
                                                                 parameterization,
                                                                 curve_degrees,
                                                                 closed_curve),
                              std::invalid_argument);
        }


        SECTION("parameterization out of range") {
            vector<vector<VectorXd>> data_points = {{Scalar(0.232), Scalar(0.43), Scalar(0.3434), Scalar(0.343), Scalar(
                    0.123), Scalar(-0.32)}};
            vector<vector<double>> parameterization = {{0, 0.1, 0.4, 0.5, 2}};
            vector<int> curve_degrees = {5};
            bool closed_curve = false;

            REQUIRE_THROWS_AS(bezier::fit_composite_bezier_curve(data_points,
                                                                 parameterization,
                                                                 curve_degrees,
                                                                 closed_curve),
                              std::invalid_argument);
        }
        SECTION("ambigiuous paramterization"){

            vector<vector<VectorXd>> data_points = {{Vector3d(2, 1, 0), Vector3d(-1, 3, 2), Vector3d(10, 2, -1), Vector3d(5, 3, 2)},
                                                    {Vector3d(5, 3, 2), Vector3d(1, 0, 0)},
                                                    {Vector3d(1, 0, 0), Vector3d(5, 4, 3)}};
            vector<vector<double>> parameterization = {{0, 0.25, 0.75, 1}, {0, 1}, {0.25, 0.75, 1}};
            vector<int> curve_degrees = {3, 3, 3};
            bool closed_curve = false;

            REQUIRE_THROWS_AS(bezier::fit_composite_bezier_curve(data_points,
                                                                 parameterization,
                                                                 curve_degrees,
                                                                 closed_curve),
                              std::invalid_argument);

            parameterization = {{0, 0.25, 0.75, 1}, {0.1, 1}, {0.25, 0.75, 1}};
            closed_curve = true;

            REQUIRE_THROWS_AS(bezier::fit_composite_bezier_curve(data_points,
                                                                 parameterization,
                                                                 curve_degrees,
                                                                 closed_curve),
                              std::invalid_argument);
        }



    }

    SECTION("interpolation"){

        SECTION("single curve"){

            SECTION("open curve"){
                vector<vector<VectorXd>> data_points = {{Vector2d(2, 1), Vector2d(-1, 3), Vector2d(10, 2), Vector2d(5, 3)}};
                vector<vector<double>> parameterization = {{0, 0.25, 0.75, 1}};
                vector<int> curve_degrees = {1};
                bool closed_curve = false;

                REQUIRE_NOTHROW(bezier::fit_composite_bezier_curve(data_points,
                                                                   parameterization,
                                                                   curve_degrees,
                                                                   closed_curve));


                data_points = {{Vector2d(2, 1), Vector2d(-1, 3), Vector2d(10, 2), Vector2d(5, 3)}};
                parameterization = {{0, 0.25, 0.75, 1}};
                curve_degrees = {3};
                closed_curve = false;

                bezier::CompositeBezierCurve curve = bezier::fit_composite_bezier_curve(data_points,
                                                                                        parameterization,
                                                                                        curve_degrees,
                                                                                        closed_curve);
                REQUIRE(curve.dimension() == 2);
                REQUIRE(curve(parameterization[0][0]).isApprox(data_points[0][0]));
                REQUIRE(curve(parameterization[0][1]).isApprox(data_points[0][1]));
                REQUIRE(curve(parameterization[0][2]).isApprox(data_points[0][2]));
                REQUIRE(curve(parameterization[0][3]).isApprox(data_points[0][3]));


                data_points = {{Scalar(0.232), Scalar(0.43), Scalar(0.3434), Scalar(0.343), Scalar(0.123), Scalar(-0.32)}};
                parameterization = {{0, 0.75, 0.4, 0.5, 0.1, 0.9}};
                curve_degrees = {5};
                closed_curve = false;
                curve = bezier::fit_composite_bezier_curve(data_points,
                                                           parameterization,
                                                           curve_degrees,
                                                           closed_curve);
                REQUIRE(curve.dimension() == 1);
                for (int i = 0; i < data_points.size(); ++i) {
                    for (int j = 0; j < data_points[i].size(); ++j) {
                        REQUIRE(curve((i + parameterization[i][j]) / ((double) data_points.size()))
                                        .isApprox(data_points[i][j], 1e-8));
                    }
                }
            }

            SECTION("closed curve"){
                vector<vector<VectorXd>> data_points = {{Scalar(0.232), Scalar(0.43), Scalar(0.3434), Scalar(0.343)}};
                vector<vector<double>> parameterization = {{0, 0.75, 0.4, 0.9}};
                vector<int> curve_degrees = {5};
                bool closed_curve = true;
                bezier::CompositeBezierCurve curve = bezier::fit_composite_bezier_curve(data_points,
                                                                                        parameterization,
                                                                                        curve_degrees,
                                                                                        closed_curve);

                REQUIRE(curve.dimension() == 1);
                for (int i = 0; i < data_points.size(); ++i) {
                    for (int j = 0; j < data_points[i].size(); ++j) {
                        REQUIRE(curve((i + parameterization[i][j]) / ((double) data_points.size()))
                                        .isApprox(data_points[i][j], 1e-8));
                    }
                }
                REQUIRE(curve(0) == curve(1));

                parameterization = {{0.3, 0.75, 0.4, 1}};
                curve = bezier::fit_composite_bezier_curve(data_points,
                                                           parameterization,
                                                           curve_degrees,
                                                           closed_curve);

                REQUIRE(curve.dimension() == 1);
                for (int i = 0; i < data_points.size(); ++i) {
                    for (int j = 0; j < data_points[i].size(); ++j) {
                        REQUIRE(curve((i + parameterization[i][j]) / ((double) data_points.size()))
                                        .isApprox(data_points[i][j], 1e-8));
                    }
                }
                REQUIRE(curve(0) == curve(1));

            }

        }

        SECTION("composite curve"){

            SECTION("open curve"){
                vector<vector<VectorXd>> data_points = {{Vector3d(2, 1, 0), Vector3d(-1, 3, 2), Vector3d(10, 2, -1), Vector3d(5, 3, 2)},
                                                        {Vector3d(5, 3, 2), Vector3d(1, 0, 0)},
                                                        {Vector3d(1, 0, 0), Vector3d(-1, -2, -3), Vector3d(5, 4, 3)}};
                vector<vector<double>> parameterization = {{0, 0.75, 0.25, 1}, {0.25, 1}, {0.25, 0.75, 1}};
                vector<int> curve_degrees = {3, 3, 4};
                bool closed_curve = false;

                bezier::CompositeBezierCurve curve = bezier::fit_composite_bezier_curve(data_points,
                                                                                        parameterization,
                                                                                        curve_degrees,
                                                                                        closed_curve);
                REQUIRE(curve.dimension() == 3);
                for (int i = 0; i < data_points.size(); ++i) {
                    for (int j = 0; j < data_points[i].size(); ++j) {
                        REQUIRE(curve((i + parameterization[i][j]) / ((double) data_points.size())).isApprox(data_points[i][j]));
                    }
                }
            }

            SECTION("closed curve"){
                vector<vector<VectorXd>> data_points = {{Vector3d(2, 1, 0), Vector3d(-1, 3, 2)},
                                                        {Vector3d(5, 3, 2), Vector3d(1, 0, 0)},
                                                        {Vector3d(1, 0, 0), Vector3d(-1, -2, -3), Vector3d(5, 4, 3)}};
                vector<vector<double>> parameterization = {{0, 0.75}, {0.25, 1}, {0.25, 0.75, 0.9}};
                vector<int> curve_degrees = {3, 3, 4};
                bool closed_curve = true;

                bezier::CompositeBezierCurve curve = bezier::fit_composite_bezier_curve(data_points,
                                                                                        parameterization,
                                                                                        curve_degrees,
                                                                                        closed_curve);
                REQUIRE(curve.dimension() == 3);
                for (int i = 0; i < data_points.size(); ++i) {
                    for (int j = 0; j < data_points[i].size(); ++j) {
                        REQUIRE(curve((i + parameterization[i][j]) / ((double) data_points.size()))
                                        .isApprox(data_points[i][j], 1e-8));
                    }
                }
                REQUIRE(curve(0) == curve(1));

                data_points = {{Vector3d(2, 1, 0), Vector3d(-1, 3, 2)},
                               {Vector3d(1, 0, 0), Vector3d(-1, -2, -3), Vector3d(5, 4, 3)}};
                parameterization = {{0, 0.75}, {0.25, 0.75, 0.9}};
                curve_degrees = {3, 4};
                closed_curve = true;

                curve = bezier::fit_composite_bezier_curve(data_points,
                                                           parameterization,
                                                           curve_degrees,
                                                           closed_curve);

                REQUIRE(curve.dimension() == 3);
                for (int i = 0; i < data_points.size(); ++i) {
                    for (int j = 0; j < data_points[i].size(); ++j) {
                        REQUIRE(curve((i + parameterization[i][j]) / ((double) data_points.size()))
                                        .isApprox(data_points[i][j], 1e-8));
                    }
                }
                REQUIRE(curve(0) == curve(1));
            }
        }

    }


    SECTION("least squares fitting"){
        vector<vector<VectorXd>> data_points = {{Vector2d(2, 1), Vector2d(-1, 3), Vector2d(10, 2), Vector2d(5, 3),
                                                Vector2d(-2, 5), Vector2d(-10, 10), Vector2d(3, 3)}};
        vector<vector<double>> parameterization = {{0, 0.25, 0.3, 0.4, 0.6, 0.7, 1}};
        vector<int> curve_degrees = {3};
        bool closed_curve = false;

        REQUIRE_NOTHROW(bezier::fit_composite_bezier_curve(data_points,
                                                           parameterization,
                                                           curve_degrees,
                                                           closed_curve));

        data_points = {{Vector3d(2, 1, 0), Vector3d(-1, 3, 2), Vector3d(10, 2, -1), Vector3d(5, 3, 2)},
                       {Vector3d(5, 3, 2), Vector3d(1, 0, 0), Vector3d(-23, 10.3, 5), Vector3d(20, 20, 10)},
                       {Vector3d(20, 10, 10), Vector3d(-1, -2, -3), Vector3d(5, 4, 3)}};
        parameterization = {{0, 0.25, 0.75, 1}, {0.1, 0.25, 0.75, 1}, {0.25, 0.75, 1}};
        curve_degrees = {3, 3, 4};
        closed_curve = false;

        REQUIRE_NOTHROW(bezier::fit_composite_bezier_curve(data_points,
                                                           parameterization,
                                                           curve_degrees,
                                                           closed_curve));
    }


}

