cmake_minimum_required(VERSION 2.8)
project(ORB_SLAM2)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall -O3 -march=native -g")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3 -march=native -g -std=c++11")

add_definitions(-DCOMPILEDWITHC11)

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(OpenCV 2.4.3 REQUIRED)
find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Protobuf REQUIRED)
find_package(AVFormat REQUIRED)

include_directories(
${CMAKE_CURRENT_BINARY_DIR}
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${EIGEN3_INCLUDE_DIR}
${AVFormat_INCLUDE_DIRS}
${Pangolin_INCLUDE_DIRS}
${PROTOBUF_INCLUDE_DIRS}
${PROJECT_SOURCE_DIR}/../DBoW2
${PROJECT_SOURCE_DIR}/../g2o
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

file(GLOB ProtoFiles "src/proto/*.proto")
# TODO: make the generated files go somewhere other than the root
# directory.
PROTOBUF_GENERATE_CPP(ProtoSources ProtoHeaders ${ProtoFiles})

add_library(${PROJECT_NAME} SHARED
src/System.cc
src/Tracking.cc
src/LocalMapping.cc
src/LoopClosing.cc
src/ORBextractor.cc
src/ORBmatcher.cc
src/ORBVocabulary.cc
src/FrameDrawer.cc
src/Converter.cc
src/MapPoint.cc
src/MapPointersIndex.cc
src/KeyFrame.cc
src/Map.cc
src/MapDrawer.cc
src/Optimizer.cc
src/PnPsolver.cc
src/Frame.cc
src/KeyFrameDatabase.cc
src/SerializationHelpers.cc
src/Sim3Solver.cc
src/Initializer.cc
src/Viewer.cc
${ProtoSources}
${ProtoHeaders}
)

add_dependencies(${PROJECT_NAME} g2o DBoW2)
target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
DBoW2
g2o
glog
${PROTOBUF_LIBRARY}
)
