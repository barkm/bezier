#include <catch/catch.hpp>

#include <bezier/postscript/postscript.h>

using Eigen::Vector2d;
using std::string;

TEST_CASE("Postscript circle test", "[pscircle]"){
    string circle = bezier::ps_circle(Vector2d(10, 2), 2, true);
    REQUIRE(circle == "1 setlinewidth 10 2 2 0 360 arc fill\n");
}

TEST_CASE("Postscript line test", "[psline]"){
    string line = bezier::ps_line(Vector2d(3, 4), Vector2d(10 ,5), 2);
    REQUIRE(line == "2 setlinewidth 3 4 moveto 10 5 lineto stroke\n");
    string dashed_line = bezier::ps_line(Vector2d(3, 4), Vector2d(10 ,5), 2, true);
    REQUIRE(dashed_line == "2 setlinewidth [3 3] 0 setdash 3 4 moveto 10 5 lineto stroke\n[] 0 setdash\n");
}

TEST_CASE("Postscript polyline test", "[pspolyline]"){
    string polyline = bezier::ps_polyline({Vector2d(0, 0), Vector2d(3, 4), Vector2d(10, 5)});
    REQUIRE(polyline == "1 setlinewidth 0 0 moveto 3 4 lineto 10 5 lineto stroke\n");
}

TEST_CASE("Postscript cuibc test", "[pscubic]"){
    std::array<Vector2d, 4> control_points = { Vector2d(1, -1),
                                               Vector2d(-1, 3),
                                               Vector2d(4, 10),
                                               Vector2d(1, -2)};
    string cubic = bezier::ps_cubic(control_points);
    REQUIRE(cubic == "1 setlinewidth 1 -1 moveto -1 3 4 10 1 -2 curveto stroke\n");
}

TEST_CASE("Postscript header test", "[psheader]"){
    string header = bezier::ps_header({Vector2d(0, 0), Vector2d(10, 10)});
    REQUIRE(header == "%!PS-Adobe-3.0 EPSF-3.0\n%%BoundingBox: 0 0 10 10\n");
}
