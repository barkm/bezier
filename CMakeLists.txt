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
        include/bezier/postscript/postscript.h
        src/postscript/postscript.cpp)


add_library(bezier STATIC ${SOURCE_FILES})

include_directories(test/third-party)

add_executable(bezier_curve_test test/bezier_curve_test.cpp)
target_link_libraries(bezier_curve_test bezier)

add_executable(composite_bezier_curve_test test/composite_bezier_curve_test.cpp)
target_link_libraries(composite_bezier_curve_test bezier)

add_executable(fit_composite_bezier_curve_test test/fit_composite_bezier_curve_test.cpp)
target_link_libraries(fit_composite_bezier_curve_test bezier)

add_executable(math_test test/math_test.cpp)
target_link_libraries(math_test bezier)

add_executable(utilities_test test/utilities_test.cpp)
target_link_libraries(utilities_test bezier)

add_executable(postscript_test test/postscript_test.cpp)
target_link_libraries(postscript_test bezier)

add_executable(curve_test test/curve_test.cpp)
target_link_libraries(curve_test bezier)

add_executable(main main.cpp)
target_link_libraries(main bezier)
