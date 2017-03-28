cmake_minimum_required(VERSION 2.8)
project(pilotguru)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall -O3 -march=native -g")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3 -march=native -g -std=c++11")

add_subdirectory(thirdparty)

include_directories(
${PROJECT_SOURCE_DIR}/include
${PROJECT_SOURCE_DIR}/thirdparty/orb-slam2/include
${PROJECT_SOURCE_DIR}/thirdparty/DBoW2
${PROJECT_SOURCE_DIR}/thirdparty/g2o
${EIGEN3_INCLUDE_DIR}
${CMAKE_CURRENT_BINARY_DIR}/thirdparty/orb-slam2  # For generated proto files.
)

find_package(OpenCV 2.4.3 REQUIRED)

add_library(pilotgurulib SHARED
  ${PROJECT_SOURCE_DIR}/src/io/image_sequence_reader.cc
  ${PROJECT_SOURCE_DIR}/src/io/image_sequence_writer.cc
  ${PROJECT_SOURCE_DIR}/src/slam/track_image_sequence.cc
)

add_executable(calibrate src/calibrate.cc)
add_executable(optical_trajectories src/optical_trajectories.cc)
add_executable(render_turning src/render_turning.cc)

add_dependencies(pilotgurulib ORB_SLAM2)
add_dependencies(optical_trajectories pilotgurulib ORB_SLAM2)
add_dependencies(render_turning pilotgurulib ORB_SLAM2)

target_link_libraries(calibrate ${OpenCV_LIBS} glog gflags)
target_link_libraries(optical_trajectories pilotgurulib ORB_SLAM2 ${OpenCV_LIBS} glog gflags)
target_link_libraries(render_turning pilotgurulib ORB_SLAM2 ${OpenCV_LIBS} glog gflags)
