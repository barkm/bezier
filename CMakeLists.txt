cmake_minimum_required(VERSION 3.6)
project(bezier)

set(CMAKE_CXX_STANDARD 11)

include_directories(include)
include_directories(/usr/local/include/eigen3/)

add_definitions(-DCMAKE_BINARY_DIR=${CMAKE_BINARY_DIR})

set(SOURCE_FILES
        include/bezier/bezier_curve.h
        src/math/misc.cpp
        include/bezier/math/misc.h
        include/bezier/curve.h
        src/curve.cpp
        src/bezier_curve.cpp
        include/bezier/composite_bezier_curve.h
        src/composite_bezier_curve.cpp
        src/math/tridiagonal.cpp
        include/bezier/math/tridiagonal.h
        src/fit_composite_bezier_curve.cpp
        include/bezier/fit_composite_bezier_curve.h
        include/bezier/utilities.h
        include/bezier/postscript/bezier_postscript.h
        src/postscript/bezier_postscript.cpp
        include/bezier/postscript/postscript.h
        src/postscript/postscript.cpp include/bezier/bezier.h)


add_library(bezier STATIC ${SOURCE_FILES})

include_directories(test/third-party)

add_executable(tests
        test/main_test.cpp
        test/bezier_curve_test.cpp
        test/composite_bezier_curve_test.cpp
        test/fit_composite_bezier_curve_test.cpp
        test/math_test.cpp
        test/utilities_test.cpp
        test/curve_test.cpp
        test/postscript_test.cpp)
target_link_libraries(tests bezier)

add_executable(circles examples/circles.cpp)
target_link_libraries(circles bezier)

add_executable(main main.cpp)
target_link_libraries(main bezier)
