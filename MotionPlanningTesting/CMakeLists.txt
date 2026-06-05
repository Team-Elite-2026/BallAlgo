cmake_minimum_required(VERSION 3.16)
project(MotionPlanningTesting CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

add_executable(motion_planning
    src/main.cpp
    src/kalman.cpp
    src/ball_kalman.cpp
    src/robot_ekf.cpp
    src/astar.cpp
    src/hermite_spline.cpp
    src/velocity_profiler.cpp
)

target_include_directories(motion_planning PRIVATE src)
target_link_libraries(motion_planning PRIVATE Eigen3::Eigen)

target_compile_options(motion_planning PRIVATE -Wall -Wextra -O2)
