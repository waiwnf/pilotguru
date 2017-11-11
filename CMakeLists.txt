cmake_minimum_required(VERSION 2.8)
project(pilotguru)

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

find_package(Qt5Widgets REQUIRED)
find_package(OpenCV 2.4.3 REQUIRED)

# For QT GUI projects, run MOC and UIC automatically when needed.
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall -O3 -march=native -g")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3 -march=native -g -std=c++11")

enable_testing()

include(ExternalProject)

# ---------------------- Google test start ----------------------
# Adaped from http://www.kaizou.org/2014/11/gtest-cmake/

# We need thread support
find_package(Threads REQUIRED)

# Download and install GoogleTest
ExternalProject_Add(
    gtest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG           master
    UPDATE_COMMAND    ""
    PREFIX ${CMAKE_CURRENT_BINARY_DIR}/gtest
    # Disable install step
    INSTALL_COMMAND ""
)

# Get GTest source and binary directories from CMake project
ExternalProject_Get_Property(gtest source_dir binary_dir)

# Create a libgtest target to be used as a dependency by test programs
add_library(libgtest_main IMPORTED STATIC GLOBAL)
add_dependencies(libgtest_main gtest)

# Set libgtest properties
set_target_properties(libgtest_main PROPERTIES
    "IMPORTED_LOCATION" "${binary_dir}/googlemock/gtest/libgtest_main.a"
    "IMPORTED_LINK_INTERFACE_LIBRARIES" "${CMAKE_THREAD_LIBS_INIT}"
)

# Create a libgmock target to be used as a dependency by test programs
add_library(libgmock IMPORTED STATIC GLOBAL)
add_dependencies(libgmock gtest)

# Set libgmock properties
set_target_properties(libgmock PROPERTIES
    "IMPORTED_LOCATION" "${binary_dir}/googlemock/libgmock.a"
    "IMPORTED_LINK_INTERFACE_LIBRARIES" "${CMAKE_THREAD_LIBS_INIT}"
)

include_directories("${source_dir}/googletest/include"
                    "${source_dir}/googlemock/include")

# ---------------------- Google test end ----------------------

ExternalProject_Add(can-utils
  PREFIX            ${CMAKE_BINARY_DIR}/can-utils
  GIT_REPOSITORY    https://github.com/waiwnf/can-utils.git
  GIT_TAG           master
  UPDATE_COMMAND    ""
  CONFIGURE_COMMAND autoreconf --install -Wall && ./configure --prefix=<INSTALL_DIR>
  BUILD_IN_SOURCE   1
)
ExternalProject_Get_Property(can-utils source_dir)
add_library(socketcanlib STATIC IMPORTED)
set_property(TARGET socketcanlib PROPERTY IMPORTED_LOCATION ${source_dir}/.libs/libcan.a)
include_directories(
  ${source_dir}/include
)

add_subdirectory(thirdparty/DBoW2)
add_subdirectory(thirdparty/g2o)
add_subdirectory(thirdparty/orb-slam2)

include_directories(
${PROJECT_SOURCE_DIR}/include
${PROJECT_SOURCE_DIR}/thirdparty/orb-slam2/include
${PROJECT_SOURCE_DIR}/thirdparty/DBoW2
${PROJECT_SOURCE_DIR}/thirdparty/g2o
${PROJECT_SOURCE_DIR}/thirdparty/LBFGS
${EIGEN3_INCLUDE_DIR}
${CMAKE_CURRENT_BINARY_DIR}/thirdparty/orb-slam2  # For generated proto files.
${PROJECT_SOURCE_DIR}/sketches/libs/kia-spoof-steering
${PROJECT_SOURCE_DIR}/sketches/libs/spoof-steering-serial-commands
)

add_library(pilotgurulib SHARED
  ${PROJECT_SOURCE_DIR}/src/calibration/rotation.cc
  ${PROJECT_SOURCE_DIR}/src/calibration/velocity.cc
  ${PROJECT_SOURCE_DIR}/src/geometry/geometry.cc
  ${PROJECT_SOURCE_DIR}/src/io/image_sequence_reader.cc
  ${PROJECT_SOURCE_DIR}/src/io/image_sequence_writer.cc
  ${PROJECT_SOURCE_DIR}/src/interpolation/align_time_series.cc
  ${PROJECT_SOURCE_DIR}/src/io/json_converters.cc
  ${PROJECT_SOURCE_DIR}/src/optimization/gradient_descent.cc
  ${PROJECT_SOURCE_DIR}/src/slam/horizontal_flatten.cc
  ${PROJECT_SOURCE_DIR}/src/slam/track_image_sequence.cc
  ${PROJECT_SOURCE_DIR}/src/slam/smoothing.cc
)
target_link_libraries(pilotgurulib ORB_SLAM2)

add_library(
    steeringcommandslib SHARED
    ${PROJECT_SOURCE_DIR}/sketches/libs/spoof-steering-serial-commands/spoof-steering-serial-commands.cpp)


add_library(arduinocommlib SHARED ${PROJECT_SOURCE_DIR}/src/car/arduino_comm.cc)
target_link_libraries(arduinocommlib steeringcommandslib)

add_library(canlib SHARED ${PROJECT_SOURCE_DIR}/src/car/can.cc)
add_dependencies(canlib can-utils)
target_link_libraries(arduinocommlib steeringcommandslib socketcanlib)

add_library(kalman_filter_lib SHARED ${PROJECT_SOURCE_DIR}/src/car/kalman_filter.cc)
target_link_libraries(kalman_filter_lib glog)

add_library(kiacanlib SHARED ${PROJECT_SOURCE_DIR}/src/car/kia_can.cc)
target_link_libraries(kiacanlib canlib glog)

add_library(
    kia_steering_angle_holder_lib SHARED
    ${PROJECT_SOURCE_DIR}/src/car/kia_steering_angle_holder.cc)
target_link_libraries(kia_steering_angle_holder_lib kalman_filter_lib kiacanlib arduinocommlib glog)

add_library(
	spoof_voltage_smoothing_lib SHARED
	${PROJECT_SOURCE_DIR}/sketches/libs/kia-spoof-steering/spoof-voltage-smoothing.cpp)

add_executable(annotate_frames src/annotate_frames.cc)
add_executable(calibrate src/calibrate.cc)
add_executable(fit_motion src/fit_motion.cc)
add_executable(integrate_motion src/integrate_motion.cc)
add_executable(interpolate_velocity src/interpolate_velocity.cc)
add_executable(kia_steering_angle src/kia_steering_angle.cc)
add_executable(kia_steering_ui src/kia_steering_ui.cc src/ui/kia_steering_ui_main_window.cpp)
add_executable(optical_trajectories src/optical_trajectories.cc)
add_executable(project_translations src/project_translations.cc)
add_executable(render_motion src/render_motion.cc)
add_executable(smooth_heading_directions src/smooth_heading_directions.cc)

add_executable(
    steeringcommandslib_test
    ${PROJECT_SOURCE_DIR}/sketches/libs/spoof-steering-serial-commands/test/spoof-steering-serial-commands_test.cpp)
target_link_libraries(steeringcommandslib_test steeringcommandslib libgtest_main libgmock)
add_test(SteeringCommandsTests steeringcommandslib_test)

add_executable(
    kia_steering_angle_holder_test
    ${PROJECT_SOURCE_DIR}/src/car/kia_steering_angle_holder_test.cc)
target_link_libraries(kia_steering_angle_holder_test kia_steering_angle_holder_lib arduinocommlib kiacanlib canlib steeringcommandslib libgtest_main libgmock glog)  
add_test(KiaSteeringAngleHolderTests kia_steering_angle_holder_test)

add_executable(
    kalman_filter_lib_test
    ${PROJECT_SOURCE_DIR}/src/car/kalman_filter_test.cc)
target_link_libraries(kalman_filter_lib_test kalman_filter_lib libgtest_main libgmock glog)  
add_test(KalmanFilterTests kalman_filter_lib_test)

add_executable(
    spoof_voltage_smoothing_lib_test
    ${PROJECT_SOURCE_DIR}/sketches/libs/kia-spoof-steering/test/spoof-voltage-smoothing_test.cpp)
target_link_libraries(spoof_voltage_smoothing_lib_test spoof_voltage_smoothing_lib libgtest_main libgmock glog)  
add_test(SpoofVoltageSmoothingLibTest spoof_voltage_smoothing_lib_test)

target_link_libraries(annotate_frames pilotgurulib ${OpenCV_LIBS} glog gflags)
target_link_libraries(calibrate ${OpenCV_LIBS} glog gflags)
target_link_libraries(fit_motion pilotgurulib glog gflags)
target_link_libraries(integrate_motion pilotgurulib glog gflags)
target_link_libraries(interpolate_velocity pilotgurulib glog gflags)
target_link_libraries(kia_steering_angle kiacanlib glog gflags pthread)
target_link_libraries(kia_steering_ui kia_steering_angle_holder_lib arduinocommlib kiacanlib glog gflags pthread Qt5::Widgets)
target_link_libraries(optical_trajectories pilotgurulib ${OpenCV_LIBS} glog gflags)
target_link_libraries(project_translations pilotgurulib ${OpenCV_LIBS} glog gflags)
target_link_libraries(render_motion pilotgurulib ${OpenCV_LIBS} glog gflags)
target_link_libraries(smooth_heading_directions pilotgurulib glog gflags)
