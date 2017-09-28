cmake_minimum_required(VERSION 3.6)
project(bezier)

set(CMAKE_CXX_STANDARD 11)

include_directories(include)
include_directories(/usr/local/include/eigen3/)


set(SOURCE_FILES main.cpp include/bezier/bezier_curve.hpp src/math/misc.cpp include/bezier/math/misc.h)

add_library(bezier STATIC ${SOURCE_FILES})

include_directories(test/third-party)

add_executable(bezier_curve_test test/bezier_curve_test.cpp)
target_link_libraries(bezier_curve_test bezier)

add_executable(math_test test/math_test.cpp)
target_link_libraries(math_test bezier)

