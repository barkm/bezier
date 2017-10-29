cmake_minimum_required(VERSION 3.6)
project(bezier)

set(CMAKE_CXX_STANDARD 11)

# Directories for library and executables
set(LIB_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(BIN_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${LIB_DIRECTORY})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${BIN_DIRECTORY})

# Include directories
include_directories(include)
include_directories(test/third-party)

# Dependencies
include_directories(/usr/local/include/eigen3/)

# Bezier library
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

# Test target
add_executable(tests
        test/main_test.cpp
        test/bezier_curve_test.cpp
        test/composite_bezier_curve_test.cpp
        test/fit_composite_bezier_curve_test.cpp
        test/math_test.cpp
        test/utilities_test.cpp
        test/curve_test.cpp
        )
target_link_libraries(tests bezier)

# Run the tests after compilation
add_custom_command(TARGET tests POST_BUILD COMMAND ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/tests)

# Examples
add_executable(example examples/example.cpp)
target_link_libraries(example bezier)

add_executable(circles examples/circles.cpp)
target_link_libraries(circles bezier)

add_executable(wave examples/wave.cpp)
target_link_libraries(wave bezier)

add_executable(interpolation examples/interpolation.cpp)
target_link_libraries(interpolation bezier)

# Install target
install(DIRECTORY include/bezier DESTINATION include)
install(FILES ${LIB_DIRECTORY}/libbezier.a DESTINATION lib)

# Uninstall target
configure_file(
        "${CMAKE_CURRENT_SOURCE_DIR}/cmake_uninstall.cmake.in"
        "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
        IMMEDIATE @ONLY)

add_custom_target(uninstall
        COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)
