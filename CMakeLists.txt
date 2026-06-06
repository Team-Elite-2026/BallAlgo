cmake_minimum_required(VERSION 3.16)
project(ballalgo CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(BALLALGO_ENABLE_LIDAR "Enable LD19 lidar" ON)
option(BALLALGO_BUILD_APP "Build production ballalgo runtime" ON)

find_package(Eigen3 REQUIRED)
set(BALLALGO_MOTION_CORE_SOURCES
  src/motion/AStar3D.cpp
  src/motion/DefensePose.cpp
  src/motion/HermiteSpline.cpp
  src/motion/OffensePose.cpp
  src/motion/VelocityProfile.cpp
  src/motion/StrikePose.cpp
  src/motion/MotionPlanner.cpp
  src/vision/VisionMath.cpp
)

add_library(ballalgo_motion_core STATIC ${BALLALGO_MOTION_CORE_SOURCES})
target_include_directories(ballalgo_motion_core PUBLIC src)
target_link_libraries(ballalgo_motion_core PUBLIC Eigen3::Eigen)
target_compile_options(ballalgo_motion_core PRIVATE -O3 -Wall -Wextra)

if(BALLALGO_BUILD_APP)
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBCAMERA REQUIRED libcamera)
pkg_check_modules(LIBGPiod libgpiod)

set(BALLALGO_APP_SOURCES
  src/main.cpp
  src/FoxGloveSim/FoxgloveConfig.cpp
  src/FoxGloveSim/FoxgloveTelemetryPublisher.cpp
  src/vision/Thresholds.cpp
  src/vision/SectorTracker.cpp
  src/vision/VisionMath.cpp
  src/camera/CameraCapture.cpp
  src/lidar/Ld19Reader.cpp
  src/lidar/LidarLocalizer.cpp
  src/estimation/PoseKalman.cpp
  src/estimation/BallKalman.cpp
  src/motion/Protocol.cpp
  src/motion/ClockSync.cpp
  src/motion/ActionChunkPublisher.cpp
  src/motion/MotionPipeline.cpp
  src/io/RobotSerial.cpp
  src/io/GpioLidar.cpp
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

if(BALLALGO_ENABLE_LIDAR)
  target_compile_definitions(ballalgo PRIVATE BALLALGO_ENABLE_LIDAR=1)
endif()

target_compile_options(ballalgo PRIVATE -O3 -Wall -Wextra)

install(TARGETS ballalgo RUNTIME DESTINATION bin)
install(FILES thresholds.json DESTINATION share/ballalgo OPTIONAL)
endif()
