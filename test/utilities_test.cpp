#include <catch/catch.hpp>

#include <vector>

#include <bezier/utilities.h>

using std::vector;

TEST_CASE("Partition data test", "[partition]"){

    vector<int> v = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
    vector<int> joints = {2, 7};

    vector<vector<int>> p = bezier::partition_data<int>(v, joints);

    REQUIRE((p[0] == vector<int>{1, 2, 3}));
    REQUIRE((p[1] == vector<int>{4, 5, 6, 7, 8}));
    REQUIRE((p[2] == vector<int>{9, 10}));
}


