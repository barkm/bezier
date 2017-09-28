cmake_minimum_required(VERSION 3.6)
project(bezier)

set(CMAKE_CXX_STANDARD 11)

include_directories(include)
include_directories(/usr/local/include/eigen3/)


set(SOURCE_FILES
        include/bezier/bezier_curve.h
        src/math/misc.cpp
        include/bezier/math/misc.h
        include/bezier/curve.h
        src/bezier_curve.cpp
        include/bezier/composite_bezier_curve.h src/composite_bezier_curve.cpp)


add_library(bezier STATIC ${SOURCE_FILES})

include_directories(test/third-party)

add_executable(bezier_curve_test test/bezier_curve_test.cpp)
target_link_libraries(bezier_curve_test bezier)

add_executable(composite_bezier_curve_test test/composite_bezier_curve_test.cpp)
target_link_libraries(composite_bezier_curve_test bezier)

add_executable(math_test test/math_test.cpp)
target_link_libraries(math_test bezier)

add_executable(main main.cpp)
