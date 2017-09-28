#define CATCH_CONFIG_MAIN
#include <catch/catch.hpp>

#include <bezier/math/misc.h>

TEST_CASE("Factorial tests", "[factorial]"){

    SECTION("test domain"){
        REQUIRE_NOTHROW(bezier::factorial(0));
        REQUIRE_NOTHROW(bezier::factorial(6));
        REQUIRE_THROWS_AS(bezier::factorial(-1), std::domain_error);
    }

    SECTION("test evaluation"){
        REQUIRE(bezier::factorial(0) == 1);
        REQUIRE(bezier::factorial(1) == 1);
        REQUIRE(bezier::factorial(3) == 6);
        REQUIRE(bezier::factorial(10) == 3628800);
    }

}
