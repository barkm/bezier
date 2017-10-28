#include <catch/catch.hpp>

#include <vector>

#include <Eigen/Dense>

#include <bezier/math/misc.h>
#include <bezier/math/tridiagonal.h>

using Eigen::MatrixXd;
using Eigen::Matrix2d;

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



TEST_CASE("Tridiagonal solver tests", "[tridiagonal]"){


    SECTION("test invalid arguments"){
        MatrixXd a1, a2, b0, b1, b2, c0, c1, d0, d1, d2;

        Eigen::Matrix<double, 6, 6> M;

        M <<    0, 3,  1, 2,  0, 0,
                2, 1,  4, 5,  0, 0,

                6, 7,  1, 2,  4, 5,
                1, 3,  9, 0,  2, 3,

                0, 0,  2, 1,  5, 2,
                0, 0,  3, 7,  9, 3;


        Eigen::Matrix<double, 6, 1> D;
        D << 0, 2, 3, 1, -9, 2;

        b0 = M.block(0, 0, 2, 2);
        b1 = M.block(2, 2, 2, 2);
        b2 = M.block(4, 4, 2, 2);

        c0 = M.block(0, 2, 2, 2);
        c1 = M.block(2, 4, 2, 2);

        a1 = M.block(2, 0, 2, 2);
        a2 = M.block(4, 2, 2, 2);

        d0 = D.block(0, 0, 2, 1);
        d1 = D.block(2, 0, 2, 1);
        d2 = D.block(4, 0, 2, 1);

        std::vector<MatrixXd> diagonal_elements = {b0, b1, b2};
        std::vector<MatrixXd> lower_diagonal_elements = {a1, a2};
        std::vector<MatrixXd> upper_diagonal_elements = {c0, c1};
        std::vector<MatrixXd> right_hand_side = {d0, d1, d2};

        REQUIRE_THROWS_AS(bezier::solve_tridiagonal({a2}, {b0, b1, b2}, {c0, c1}, {d0, d1, d2}), std::invalid_argument);

        REQUIRE_THROWS_AS(bezier::solve_tridiagonal({a1, a2}, {b1, b2}, {c0, c1}, {d0, d1, d2}), std::invalid_argument);

        REQUIRE_THROWS_AS(bezier::solve_tridiagonal({a1, a2}, {b0, b1, b2}, {c0}, {d0, d1, d2}), std::invalid_argument);

        REQUIRE_THROWS_AS(bezier::solve_tridiagonal({a1, a2}, {b0, b1, b2}, {c0, c1}, {d0, d1}), std::invalid_argument);

        REQUIRE_THROWS_AS(bezier::solve_tridiagonal({a1, a2}, {b0, b1, b2.block(0, 0, 1, 1)}, {c0, c1}, {d0, d1, d2}), std::invalid_argument);

        REQUIRE_THROWS_AS(bezier::solve_tridiagonal({a1, a2.block(0, 0, 1, 1)}, {b0, b1, b2}, {c0, c1}, {d0, d1, d2}), std::invalid_argument);

        REQUIRE_NOTHROW(bezier::solve_tridiagonal({a1, a2}, {b0, b1, b2}, {c0, c1}, {d0, d1, d2}));
    }

    SECTION("1x1 matrix system"){
        Eigen::Matrix<double, 2, 2> M;
        M <<    1, 2,
                3, 4;
        Eigen::Matrix<double, 2, 1> D;
        D <<    5, 6;

        std::vector<Eigen::MatrixXd> solution = bezier::solve_tridiagonal({}, {M}, {}, {D});
        Eigen::Matrix<double, 2, 1> eigen_solution = M.fullPivHouseholderQr().solve(D);

        REQUIRE(solution[0].isApprox(eigen_solution));
    }

    SECTION("2x2 matrix system"){
        Eigen::Matrix<double, 2, 2> M;
        M <<    1, 2,
                3, 4;
        Eigen::Matrix<double, 2, 1> D;
        D <<    5, 6;

        Eigen::Matrix<double, 1, 1> a1, b0, b1, c0, d0, d1;
        a1 = M.block(1, 0, 1, 1);
        b0 = M.block(0, 0, 1, 1);
        b1 = M.block(1, 1, 1, 1);
        c0 = M.block(0, 1, 1, 1);

        d0 = D.block(0, 0, 1, 1);
        d1 = D.block(1, 0, 1, 1);

        std::vector<Eigen::MatrixXd> solution = bezier::solve_tridiagonal({a1}, {b0, b1}, {c0}, {d0, d1});

        Eigen::Matrix<double, 2, 1> bezier_solution;
        bezier_solution << solution[0](0,0), solution[1](0,0);

        Eigen::Matrix<double, 2, 1> eigen_solution = M.fullPivHouseholderQr().solve(D);

        REQUIRE(bezier_solution.isApprox(eigen_solution));
    }

    SECTION("3x3 matrix system") {
        Eigen::Matrix<double, 3, 3> M;
        M <<    1, 2, 0,
                3, 4, 5,
                0, 6, 7;
        Eigen::Matrix<double, 3, 1> D;
        D << 8, 9, 10;

        Eigen::Matrix<double, 1, 1> a1, a2,  b0, b1, b2, c0, c1, d0, d1, d2;
        a1 = M.block(1, 0, 1, 1);
        a2 = M.block(2, 1, 1, 1);

        b0 = M.block(0, 0, 1, 1);
        b1 = M.block(1, 1, 1, 1);
        b2 = M.block(2, 2, 1, 1);

        c0 = M.block(0, 1, 1, 1);
        c1 = M.block(1, 2, 1, 1);

        d0 = D.block(0, 0, 1, 1);
        d1 = D.block(1, 0, 1, 1);
        d2 = D.block(2, 0, 1, 1);

        std::vector<Eigen::MatrixXd> solution = bezier::solve_tridiagonal({a1, a2}, {b0, b1, b2}, {c0, c1}, {d0, d1, d2});

        Eigen::Matrix<double, 3, 1> bezier_solution;
        bezier_solution << solution[0](0, 0), solution[1](0, 0), solution[2](0,0);

        Eigen::Matrix<double, 3, 1> eigen_solution = M.fullPivHouseholderQr().solve(D);

        REQUIRE(bezier_solution.isApprox(eigen_solution));
    }

    SECTION("2x2 block matrix (2x2) system"){
        Eigen::Matrix<double, 4, 4> M;
        M <<    1, 2, 3, -4,
                -1, 6, 7, 8,
                9, 2, 11, -12,
                13, 14, 5, 16;
        Eigen::Matrix<double, 4, 1> D;
        D << 17, 18, 19, 20;

        Eigen::Matrix<double, 2, 2> a1, b0, b1, c0;
        a1 = M.block(2, 0, 2, 2);

        b0 = M.block(0, 0, 2, 2);
        b1 = M.block(2, 2, 2, 2);

        c0 = M.block(0, 2, 2, 2);

        Eigen::Matrix<double, 2, 1> d0, d1;
        d0 = D.block(0, 0, 2, 1);
        d1 = D.block(2, 0, 2, 1);

        std::vector<Eigen::MatrixXd> solution = bezier::solve_tridiagonal({a1}, {b0, b1}, {c0}, {d0, d1});

        Eigen::Matrix<double, 4, 1> bezier_solution;
        bezier_solution.block(0, 0, 2, 1) = solution[0];
        bezier_solution.block(2, 0, 2, 1) = solution[1];

        Eigen::Matrix<double, 4, 1> eigen_solution = M.fullPivHouseholderQr().solve(D);

        REQUIRE(bezier_solution.isApprox(eigen_solution));

    }

    SECTION("2x2 block matrix (different sizes) system"){
        Eigen::Matrix<double, 5, 5> M;
        M <<    1, 2,       3, -4, 2,
                -1, 6,      7, 8, 5,

                9, 2,       11, -12, 9,
                13, 14,     5, 16, 1,
                3, 2,       0, -1, 3;

        Eigen::Matrix<double, 5, 2> D;
        D << 17, 18, 19, 20, 2,
                23, 43, -2, 5, 7;

        Eigen::Matrix<double, 3, 2> a1 = M.block(2, 0, 3, 2);

        Eigen::Matrix2d b0 = M.block(0, 0, 2, 2);
        Eigen::Matrix3d b1 = M.block(2, 2, 3, 3);

        Eigen::Matrix<double, 2, 3> c0 = M.block(0, 2, 2, 3);

        Eigen::Matrix<double, 2, 2> d0 = D.block(0, 0, 2, 2);
        Eigen::Matrix<double, 3, 2> d1 = D.block(2, 0, 3, 2);

        std::vector<Eigen::MatrixXd> solution = bezier::solve_tridiagonal({a1}, {b0, b1}, {c0}, {d0, d1});

        Eigen::Matrix<double, 5, 2> bezier_solution;
        bezier_solution.block(0, 0, 2, 2) = solution[0];
        bezier_solution.block(2, 0, 3, 2) = solution[1];

        Eigen::Matrix<double, 5, 2> eigen_solution = M.fullPivHouseholderQr().solve(D);

        REQUIRE(bezier_solution.isApprox(eigen_solution));

    }

    SECTION("3x3 block matrix (2x2) system"){
        //
        //  | b0  c0  0  | |x1|   |d0|
        //  | a1  b1  c1 | |x2| = |d1|
        //  | 0   a2  b2 | |x3|   |d2|
        //
        //
        //

        MatrixXd a1, a2, b0, b1, b2, c0, c1, d0, d1, d2;

        Eigen::Matrix<double, 6, 6> M;

        M <<    0, 3,  1, 2,  0, 0,
                2, 1,  4, 5,  0, 0,

                6, 7,  1, 2,  4, 5,
                1, 3,  9, 0,  2, 3,

                0, 0,  2, 1,  5, 2,
                0, 0,  3, 7,  9, 3;


        Eigen::Matrix<double, 6, 1> D;
        D << 0, 2, 3, 1, -9, 2;

        b0 = M.block(0, 0, 2, 2);
        b1 = M.block(2, 2, 2, 2);
        b2 = M.block(4, 4, 2, 2);

        c0 = M.block(0, 2, 2, 2);
        c1 = M.block(2, 4, 2, 2);

        a1 = M.block(2, 0, 2, 2);
        a2 = M.block(4, 2, 2, 2);

        d0 = D.block(0, 0, 2, 1);
        d1 = D.block(2, 0, 2, 1);
        d2 = D.block(4, 0, 2, 1);

        std::vector<MatrixXd> diagonal_elements = {b0, b1, b2};
        std::vector<MatrixXd> lower_diagonal_elements = {a1, a2};
        std::vector<MatrixXd> upper_diagonal_elements = {c0, c1};
        std::vector<MatrixXd> right_hand_side = {d0, d1, d2};

        std::vector<MatrixXd> solution = bezier::solve_tridiagonal(lower_diagonal_elements,
                                                                   diagonal_elements,
                                                                   upper_diagonal_elements,
                                                                   right_hand_side);
        Eigen::Matrix<double, 6, 1> bezier_solution;
        bezier_solution.block(0, 0, 2, 1) = solution[0];
        bezier_solution.block(2, 0, 2, 1) = solution[1];
        bezier_solution.block(4, 0, 2, 1) = solution[2];

        Eigen::Matrix<double, 6, 1> eigen_solution = M.fullPivHouseholderQr().solve(D);

        REQUIRE(bezier_solution.isApprox(eigen_solution));
    }


}

TEST_CASE("Off tridiagonal solver tests", "[offtridiagonal]"){

    SECTION("test invalid arguments"){
        MatrixXd a0, a1, a2, b0, b1, b2, c0, c1, c2, d0, d1, d2;

        Eigen::Matrix<double, 6, 6> M;

        M <<    0, 3,  1, 2,  2, -1,
                2, 1,  4, 5,  5, 9,

                6, 7,  1, 2,  4, 5,
                1, 3,  9, 0,  2, 3,

                2, -1,  2, 1,  5, 2,
                4, 3,  3, 7,  9, 3;


        Eigen::Matrix<double, 6, 1> D;
        D << 0, 2, 3, 1, -9, 2;

        b0 = M.block(0, 0, 2, 2);
        b1 = M.block(2, 2, 2, 2);
        b2 = M.block(4, 4, 2, 2);

        c0 = M.block(0, 2, 2, 2);
        c1 = M.block(2, 4, 2, 2);
        c2 = M.block(4, 0, 2, 2);

        a0 = M.block(0, 4, 2, 2);
        a1 = M.block(2, 0, 2, 2);
        a2 = M.block(4, 2, 2, 2);

        d0 = D.block(0, 0, 2, 1);
        d1 = D.block(2, 0, 2, 1);
        d2 = D.block(4, 0, 2, 1);

        std::vector<MatrixXd> diagonal_elements = {b0, b1, b2};
        std::vector<MatrixXd> lower_diagonal_elements = {a0, a1, a2};
        std::vector<MatrixXd> upper_diagonal_elements = {c0, c1, c2};
        std::vector<MatrixXd> right_hand_side = {d0, d1, d2};

        REQUIRE_THROWS_AS(bezier::solve_off_tridiagonal({a0, a1}, {b0, b1, b2}, {c0, c1, c2}, {d0, d1, d2}), std::invalid_argument);

        REQUIRE_THROWS_AS(bezier::solve_off_tridiagonal({a0, a1, a2}, {b0, b1}, {c0, c1, c2}, {d0, d1, d2}), std::invalid_argument);

        REQUIRE_THROWS_AS(bezier::solve_off_tridiagonal({a0, a1, a2}, {b0, b1, b2}, {c0, c1}, {d0, d1, d2}), std::invalid_argument);

        REQUIRE_THROWS_AS(bezier::solve_off_tridiagonal({a0, a1, a2}, {b0, b1, b2}, {c0, c1, c2}, {d0, d1}), std::invalid_argument);

        REQUIRE_THROWS_AS(bezier::solve_off_tridiagonal({a0, a1.block(0, 0, 1, 1), a2}, {b0, b1, b2}, {c0, c1, c2}, {d0, d1, d2}), std::invalid_argument);

        REQUIRE_THROWS_AS(bezier::solve_off_tridiagonal({a0, a1, a2}, {b0, b1, b2.block(0, 0, 1, 1)}, {c0, c1, c2}, {d0, d1, d2}), std::invalid_argument);

        REQUIRE_THROWS_AS(bezier::solve_off_tridiagonal({a0, a1, a2}, {b0, b1, b2}, {c0, c1, c2.block(0, 0, 1, 1)}, {d0, d1, d2}), std::invalid_argument);

        REQUIRE_THROWS_AS(bezier::solve_off_tridiagonal({a0, a1, a2}, {b0, b1, b2}, {c0, c1, c2}, {d0, d1.block(0, 0, 1, 1), d2}), std::invalid_argument);

        REQUIRE_NOTHROW(bezier::solve_off_tridiagonal({a0, a1, a2}, {b0, b1, b2}, {c0, c1, c2}, {d0, d1, d2}));
    }

    SECTION("1x1 matrix system"){
        Eigen::Matrix<double, 2, 2> M;
        M <<    1, 2,
                3, 4;
        Eigen::Matrix<double, 2, 1> D;
        D <<    5, 6;

        std::vector<Eigen::MatrixXd> solution = bezier::solve_off_tridiagonal({}, {M}, {}, {D});
        Eigen::Matrix<double, 2, 1> eigen_solution = M.fullPivHouseholderQr().solve(D);

        REQUIRE(solution[0].isApprox(eigen_solution));
    }

    SECTION("2x2 matrix system"){
        Eigen::Matrix<double, 2, 2> M;
        M <<    1, 2,
                3, 4;
        Eigen::Matrix<double, 2, 1> D;
        D <<    5, 6;

        Eigen::Matrix<double, 1, 1> a1, b0, b1, c0, d0, d1;
        a1 = M.block(1, 0, 1, 1);
        b0 = M.block(0, 0, 1, 1);
        b1 = M.block(1, 1, 1, 1);
        c0 = M.block(0, 1, 1, 1);

        d0 = D.block(0, 0, 1, 1);
        d1 = D.block(1, 0, 1, 1);

        std::vector<Eigen::MatrixXd> solution = bezier::solve_off_tridiagonal({a1}, {b0, b1}, {c0}, {d0, d1});

        Eigen::Matrix<double, 2, 1> bezier_solution;
        bezier_solution << solution[0](0,0), solution[1](0,0);

        Eigen::Matrix<double, 2, 1> eigen_solution = M.fullPivHouseholderQr().solve(D);

        REQUIRE(bezier_solution.isApprox(eigen_solution));
    }



    SECTION("4x4 matrix system") {
        Eigen::Matrix<double, 4, 4> M;
        M <<    1,  2,  0,  -2,
                3,  4,  5,  0,
                0,  2,  -5, 3,
                -1, 0,  6,  7;
        Eigen::Matrix<double, 4, 1> D;
        D << 8, 9, 10, 11;

        Eigen::Matrix<double, 1, 1> a0, a1, a2, a3,  b0, b1, b2, b3, c0, c1, c2, c3, d0, d1, d2, d3;
        a0 = M.block(0, 3, 1, 1);
        a1 = M.block(1, 0, 1, 1);
        a2 = M.block(2, 1, 1, 1);
        a3 = M.block(3, 2, 1, 1);

        b0 = M.block(0, 0, 1, 1);
        b1 = M.block(1, 1, 1, 1);
        b2 = M.block(2, 2, 1, 1);
        b3 = M.block(3, 3, 1, 1);

        c0 = M.block(0, 1, 1, 1);
        c1 = M.block(1, 2, 1, 1);
        c2 = M.block(2, 3, 1, 1);
        c3 = M.block(3, 0, 1, 1);

        d0 = D.block(0, 0, 1, 1);
        d1 = D.block(1, 0, 1, 1);
        d2 = D.block(2, 0, 1, 1);
        d3 = D.block(3, 0, 1, 1);

        std::vector<Eigen::MatrixXd> solution = bezier::solve_off_tridiagonal({a0, a1, a2, a3}, {b0, b1, b2, b3}, {c0, c1, c2, c3}, {d0, d1, d2, d3});

        Eigen::Matrix<double, 4, 1> bezier_solution;
        bezier_solution << solution[0](0, 0), solution[1](0, 0), solution[2](0,0), solution[3](0,0);

        Eigen::Matrix<double, 4, 1> eigen_solution = M.fullPivHouseholderQr().solve(D);

        REQUIRE(bezier_solution.isApprox(eigen_solution));
    }

    SECTION("2x2 block matrix (2x2) system"){
        Eigen::Matrix<double, 4, 4> M;
        M <<    1, 2,    3, -4,
                -1, 6,   7, 8,

                9, 2,    11, -12,
                13, 14,  5, 16;

        Eigen::Matrix<double, 4, 1> D;
        D << 17, 18, 19, 20;

        Eigen::Matrix<double, 2, 2> a1, b0, b1, c0;
        a1 = M.block(2, 0, 2, 2);

        b0 = M.block(0, 0, 2, 2);
        b1 = M.block(2, 2, 2, 2);

        c0 = M.block(0, 2, 2, 2);

        Eigen::Matrix<double, 2, 1> d0, d1;
        d0 = D.block(0, 0, 2, 1);
        d1 = D.block(2, 0, 2, 1);

        std::vector<Eigen::MatrixXd> solution = bezier::solve_off_tridiagonal({a1}, {b0, b1}, {c0}, {d0, d1});

        Eigen::Matrix<double, 4, 1> bezier_solution;
        bezier_solution.block(0, 0, 2, 1) = solution[0];
        bezier_solution.block(2, 0, 2, 1) = solution[1];

        Eigen::Matrix<double, 4, 1> eigen_solution = M.fullPivHouseholderQr().solve(D);

        REQUIRE(bezier_solution.isApprox(eigen_solution));

    }

    SECTION("2x2 block matrix (different sizes) system"){
        Eigen::Matrix<double, 5, 5> M;
        M <<    1, 2,       3, -4, 2,
                -1, 6,      7, 8, 5,

                9, 2,       11, -12, 9,
                13, 14,     5, 16, 1,
                3, 2,       0, -1, 3;

        Eigen::Matrix<double, 5, 2> D;
        D << 17, 18, 19, 20, 2,
                23, 43, -2, 5, 7;

        Eigen::Matrix<double, 3, 2> a1 = M.block(2, 0, 3, 2);

        Eigen::Matrix2d b0 = M.block(0, 0, 2, 2);
        Eigen::Matrix3d b1 = M.block(2, 2, 3, 3);

        Eigen::Matrix<double, 2, 3> c0 = M.block(0, 2, 2, 3);

        Eigen::Matrix<double, 2, 2> d0 = D.block(0, 0, 2, 2);
        Eigen::Matrix<double, 3, 2> d1 = D.block(2, 0, 3, 2);

        std::vector<Eigen::MatrixXd> solution = bezier::solve_off_tridiagonal({a1}, {b0, b1}, {c0}, {d0, d1});

        Eigen::Matrix<double, 5, 2> bezier_solution;
        bezier_solution.block(0, 0, 2, 2) = solution[0];
        bezier_solution.block(2, 0, 3, 2) = solution[1];

        Eigen::Matrix<double, 5, 2> eigen_solution = M.fullPivHouseholderQr().solve(D);

        REQUIRE(bezier_solution.isApprox(eigen_solution));

    }


    SECTION("3x3 block matrix (2x2) system"){

        MatrixXd a0, a1, a2, b0, b1, b2, c0, c1, c2, d0, d1, d2;

        Eigen::Matrix<double, 6, 6> M;

        M <<    0, 3,  1, 2,  2, -1,
                2, 1,  4, 5,  5, 9,

                6, 7,  1, 2,  4, 5,
                1, 3,  9, 0,  2, 3,

                2, -1,  2, 1,  5, 2,
                4, 3,  3, 7,  9, 3;


        Eigen::Matrix<double, 6, 1> D;
        D << 0, 2, 3, 1, -9, 2;

        b0 = M.block(0, 0, 2, 2);
        b1 = M.block(2, 2, 2, 2);
        b2 = M.block(4, 4, 2, 2);

        c0 = M.block(0, 2, 2, 2);
        c1 = M.block(2, 4, 2, 2);
        c2 = M.block(4, 0, 2, 2);

        a0 = M.block(0, 4, 2, 2);
        a1 = M.block(2, 0, 2, 2);
        a2 = M.block(4, 2, 2, 2);

        d0 = D.block(0, 0, 2, 1);
        d1 = D.block(2, 0, 2, 1);
        d2 = D.block(4, 0, 2, 1);

        std::vector<MatrixXd> diagonal_elements = {b0, b1, b2};
        std::vector<MatrixXd> lower_diagonal_elements = {a0, a1, a2};
        std::vector<MatrixXd> upper_diagonal_elements = {c0, c1, c2};
        std::vector<MatrixXd> right_hand_side = {d0, d1, d2};

        std::vector<MatrixXd> solution = bezier::solve_off_tridiagonal(lower_diagonal_elements,
                                                                       diagonal_elements,
                                                                       upper_diagonal_elements,
                                                                       right_hand_side);
        Eigen::Matrix<double, 6, 1> bezier_solution;
        bezier_solution.block(0, 0, 2, 1) = solution[0];
        bezier_solution.block(2, 0, 2, 1) = solution[1];
        bezier_solution.block(4, 0, 2, 1) = solution[2];

        Eigen::Matrix<double, 6, 1> eigen_solution = M.fullPivHouseholderQr().solve(D);

        REQUIRE(bezier_solution.isApprox(eigen_solution));
    }
}


