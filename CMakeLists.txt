cmake_minimum_required(VERSION 3.16)
project(ballalgo CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(BALLALGO_ENABLE_LIDAR "Enable LD19 lidar" ON)
option(BALLALGO_BUILD_APP "Build production ballalgo runtime" ON)

find_package(Eigen3 REQUIRED)
option(BALLALGO_BUILD_TESTS "Build focused BallAlgo unit tests" ON)

set(BALLALGO_TEAM_CORE_SOURCES
  src/team/RoleArbiter.cpp
  src/team/TeamBallFilter.cpp
  src/team/TeamProto.cpp
)

set(BALLALGO_MOTION_CORE_SOURCES
  src/params.cpp
  src/motion/AStar3D.cpp
  src/motion/DefensePose.cpp
  src/motion/HermiteSpline.cpp
  src/motion/OffensePose.cpp
  src/motion/TrajectoryReplay.cpp
  src/motion/VelocityProfile.cpp
  src/motion/StrikePose.cpp
  src/motion/MotionPlanner.cpp
  src/vision/VisionMath.cpp
)

add_library(ballalgo_motion_core STATIC ${BALLALGO_MOTION_CORE_SOURCES})
target_include_directories(ballalgo_motion_core PUBLIC src)
target_link_libraries(ballalgo_motion_core PUBLIC Eigen3::Eigen)
target_compile_options(ballalgo_motion_core PRIVATE -O3 -Wall -Wextra)

set(BALLALGO_PLANNER_CASE_SOURCES
  src/FoxGloveSim/FoxgloveConfig.cpp
  src/FoxGloveSim/FoxgloveTelemetryPublisher.cpp
  src/io/RobotSerial.cpp
  src/motion/ClockSync.cpp
  src/motion/Protocol.cpp
  src/motion/TrajectoryReplayRunner.cpp
  src/planner/PlannerCaseMain.cpp
)

add_executable(ballalgo_planner_case ${BALLALGO_PLANNER_CASE_SOURCES})
target_include_directories(ballalgo_planner_case PRIVATE src)
target_link_libraries(ballalgo_planner_case PRIVATE ballalgo_motion_core Eigen3::Eigen)
target_compile_options(ballalgo_planner_case PRIVATE -O3 -Wall -Wextra)
install(TARGETS ballalgo_planner_case RUNTIME DESTINATION bin)

if(BALLALGO_BUILD_APP)
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBCAMERA REQUIRED libcamera)
pkg_check_modules(LIBGPiod libgpiod)
pkg_check_modules(BLUEZ bluez)

set(BALLALGO_APP_SOURCES
  src/main.cpp
  src/FoxGloveSim/FoxgloveConfig.cpp
  src/FoxGloveSim/FoxgloveTelemetryPublisher.cpp
  src/vision/Thresholds.cpp
  src/vision/SectorTracker.cpp
  src/vision/VisionMath.cpp
  src/camera/CameraCapture.cpp
  src/lidar/Ld19Reader.cpp
  src/lidar/LidarDeskewer.cpp
  src/lidar/LidarLocalizer.cpp
  src/estimation/PoseKalman.cpp
  src/estimation/BallKalman.cpp
  src/motion/Protocol.cpp
  src/motion/ClockSync.cpp
  src/motion/ActionChunkPublisher.cpp
  src/motion/TrajectoryReplayRunner.cpp
  src/motion/MotionPipeline.cpp
  src/io/RobotSerial.cpp
  src/io/GpioLidar.cpp
  ${BALLALGO_TEAM_CORE_SOURCES}
  src/team/TeamLink.cpp
)

add_executable(ballalgo ${BALLALGO_APP_SOURCES})
target_include_directories(ballalgo PRIVATE src ${OpenCV_INCLUDE_DIRS})

target_link_libraries(ballalgo PRIVATE ballalgo_motion_core ${OpenCV_LIBS} Eigen3::Eigen)
target_compile_definitions(ballalgo PRIVATE BALLALGO_HAS_LIBCAMERA=1)
target_include_directories(ballalgo PRIVATE ${LIBCAMERA_INCLUDE_DIRS})
target_link_libraries(ballalgo PRIVATE ${LIBCAMERA_LIBRARIES})
if(LIBGPiod_FOUND)
  target_compile_definitions(ballalgo PRIVATE BALLALGO_HAS_GPIOD=1)
  target_link_libraries(ballalgo PRIVATE ${LIBGPiod_LIBRARIES})
endif()
if(BLUEZ_FOUND)
  target_compile_definitions(ballalgo PRIVATE BALLALGO_ENABLE_BLUETOOTH=1)
  target_include_directories(ballalgo PRIVATE ${BLUEZ_INCLUDE_DIRS})
  target_link_libraries(ballalgo PRIVATE ${BLUEZ_LIBRARIES})
endif()

if(BALLALGO_ENABLE_LIDAR)
  target_compile_definitions(ballalgo PRIVATE BALLALGO_ENABLE_LIDAR=1)
endif()

target_compile_options(ballalgo PRIVATE -O3 -Wall -Wextra)

install(TARGETS ballalgo RUNTIME DESTINATION bin)
install(FILES thresholds.json DESTINATION share/ballalgo OPTIONAL)
install(FILES params.json DESTINATION share/ballalgo OPTIONAL)
endif()

if(BALLALGO_BUILD_TESTS)
enable_testing()
add_executable(ballalgo_team_tests
  tests/TeamCoordinationTests.cpp
  ${BALLALGO_TEAM_CORE_SOURCES}
  src/vision/VisionMath.cpp
)
target_include_directories(ballalgo_team_tests PRIVATE src)
target_link_libraries(ballalgo_team_tests PRIVATE Eigen3::Eigen)
target_compile_options(ballalgo_team_tests PRIVATE -O2 -Wall -Wextra)
add_test(NAME ballalgo_team_tests COMMAND ballalgo_team_tests)

add_executable(ballalgo_motion_tests
  tests/MotionPoseTests.cpp
)
target_include_directories(ballalgo_motion_tests PRIVATE src)
target_link_libraries(ballalgo_motion_tests PRIVATE ballalgo_motion_core Eigen3::Eigen)
target_compile_options(ballalgo_motion_tests PRIVATE -O2 -Wall -Wextra)
add_test(NAME ballalgo_motion_tests COMMAND ballalgo_motion_tests)

add_executable(ballalgo_protocol_tests
  tests/ProtocolTests.cpp
  src/motion/Protocol.cpp
)
target_include_directories(ballalgo_protocol_tests PRIVATE src)
target_link_libraries(ballalgo_protocol_tests PRIVATE Eigen3::Eigen)
target_compile_options(ballalgo_protocol_tests PRIVATE -O2 -Wall -Wextra)
add_test(NAME ballalgo_protocol_tests COMMAND ballalgo_protocol_tests)

add_executable(ballalgo_frame_tests
  tests/TrajectoryFrameTests.cpp
)
target_include_directories(ballalgo_frame_tests PRIVATE src)
target_link_libraries(ballalgo_frame_tests PRIVATE ballalgo_motion_core Eigen3::Eigen)
target_compile_options(ballalgo_frame_tests PRIVATE -O2 -Wall -Wextra)
add_test(NAME ballalgo_frame_tests COMMAND ballalgo_frame_tests)
endif()
