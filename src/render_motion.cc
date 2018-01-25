// From a video file and a JSON file with velocity and steering data, generates
// a video tiled with a steering wheel picture rotating according to the
// steering angle magnitude and a digital speedometer field.

#include <fstream>
#include <iostream>
#include <memory>

#include <json.hpp>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <opencv2/imgproc/imgproc.hpp>

#include <io/image_sequence_reader.hpp>
#include <io/image_sequence_writer.hpp>
#include <io/json_converters.hpp>

DEFINE_string(in_video, "", "Input video file.");
DEFINE_bool(vertical_flip, false,
            "Whether to flip the input frames vertically.");
DEFINE_bool(horizontal_flip, false,
            "Whether to flip input video frames horizontally.");

DEFINE_int32(target_video_height, -1,
             "If positive, resize the input video to this height.");
DEFINE_int32(target_video_width, -1,
             "If positive, resize the input video to this width.");

constexpr char kSteeringJsonFlagHelp[] =
    "JSON file with per-frame steering angles, produced by annotate_frames.";
constexpr char kSteeringUnitsFlagHelp[] = "Steering magnitude name in the JSON "
                                          "file, typically 'steering' or "
                                          "'angular_velocity'.";
constexpr char kSteeringScaleFlagHelp[] = "Scale to convert from raw steering "
                                          "magnitude to on-screnn steering "
                                          "wheel turn degrees.";

DEFINE_string(steering_left_json, "", kSteeringJsonFlagHelp);
DEFINE_string(steering_left_units, pilotguru::kSteering,
              kSteeringUnitsFlagHelp);
DEFINE_double(steering_left_scale, 90.0, kSteeringScaleFlagHelp);

DEFINE_string(steering_right_json, "", kSteeringJsonFlagHelp);
DEFINE_string(steering_right_units, pilotguru::kSteering,
              kSteeringUnitsFlagHelp);
DEFINE_double(steering_right_scale, 90.0, kSteeringScaleFlagHelp);

constexpr char kVelocitiesJsonFlagHelp[] =
    "JSON file with per-frame veocities in m/s, produced by annotate_frames.";
DEFINE_string(velocities_json_left, "", kVelocitiesJsonFlagHelp);
DEFINE_string(velocities_json_right, "", kVelocitiesJsonFlagHelp);

DEFINE_string(steering_wheel, "",
              "A file with the steering wheel image to use.");
DEFINE_string(out_video, "", "Output video file to write.");
DEFINE_int64(frames_to_skip, 0, "Number of initial trajectory frames to skip.");
DEFINE_int64(max_out_frames, -1, "If positive, the maximum number of frames to "
                                 "render for the output video. If the input "
                                 "trajectory has more frames, the remaining "
                                 "ones will be ignored.");

namespace {
std::unique_ptr<std::map<size_t, double>>
LoadPerFrameSeries(const std::string &json_name, const std::string &root_name,
                   const std::string &units, double scale) {
  using nlohmann::json;
  std::unique_ptr<std::map<size_t, double>> result(
      new std::map<size_t, double>());
  std::unique_ptr<json> per_frame_json = pilotguru::ReadJsonFile(json_name);
  for (const json &frame : (*per_frame_json)[root_name]) {
    (*result)[frame[pilotguru::kFrameId]] = frame[units];
    (*result)[frame[pilotguru::kFrameId]] *= scale;
  }
  return result;
}

std::unique_ptr<std::map<size_t, double>>
MaybeLoadSteering(const std::string &json_name, const std::string &units,
                  double scale) {
  if (!json_name.empty()) {
    return LoadPerFrameSeries(json_name, pilotguru::kSteering, units, scale);
  } else {
    return nullptr;
  }
}

std::unique_ptr<std::map<size_t, double>>
MaybeLoadVelocities(const std::string &json_name, double scale) {
  if (!json_name.empty()) {
    return LoadPerFrameSeries(json_name, pilotguru::kVelocities,
                              pilotguru::kSpeedMS, scale);
  } else {
    return nullptr;
  }
}

void RenderSteering(cv::Mat *out_frame, int offset_rows, int offset_cols,
                    const cv::Mat &steering_wheel, double turn_degrees) {
  CHECK_NOTNULL(out_frame);
  cv::Mat out_steer =
      out_frame->rowRange(offset_rows, offset_rows + steering_wheel.rows)
          .colRange(offset_cols, offset_cols + steering_wheel.cols);
  cv::Mat rotation_matrix = cv::getRotationMatrix2D(
      cv::Point2f(steering_wheel.cols / 2, steering_wheel.rows / 2),
      turn_degrees, 1);
  cv::warpAffine(steering_wheel, out_steer, rotation_matrix,
                 steering_wheel.size(), cv::INTER_LINEAR);
}

void MaybeRenderSteering(const std::map<size_t, double> *steering,
                         size_t frame_idx, cv::Mat *out_frame, int offset_rows,
                         int offset_cols, const cv::Mat &steering_wheel) {
  if (steering != nullptr) {
    const auto steering_it = steering->find(frame_idx);
    if (steering_it != steering->end()) {
      RenderSteering(out_frame, offset_rows, offset_cols, steering_wheel,
                     steering_it->second);
    }
  }
}

void RenderVelocity(cv::Mat *out_frame, int offset_rows, int offset_cols,
                    int window_rows, int window_cols, int velocity_km_h) {
  CHECK_NOTNULL(out_frame);
  cv::Mat out_velocity =
      out_frame->rowRange(offset_rows, offset_rows + window_rows)
          .colRange(offset_cols, offset_cols + window_cols);
  out_velocity = cv::Scalar(0, 0, 0);
  std::stringstream ss;
  ss << velocity_km_h;
  const std::string digits_text = ss.str();
  const double digits_text_scale = 3.0;
  const double units_text_scale = 0.8;
  const int text_line_thickness = 3;
  const cv::Scalar text_color(255, 255, 255);
  int baseLine = 0;
  const cv::Size digits_size =
      cv::getTextSize(digits_text, cv::FONT_HERSHEY_SIMPLEX, digits_text_scale,
                      text_line_thickness, &baseLine);
  // Velocity value.
  const int element_margin = 10;
  cv::putText(out_velocity, ss.str(),
              cv::Point(element_margin, window_rows - element_margin),
              cv::FONT_HERSHEY_SIMPLEX, digits_text_scale, text_color,
              text_line_thickness);
  // Velocity units in a smaller font.
  cv::putText(out_velocity, " km/h",
              cv::Point(element_margin + digits_size.width,
                        window_rows - element_margin),
              cv::FONT_HERSHEY_SIMPLEX, units_text_scale, text_color,
              text_line_thickness);

  // Draw the vertical speedometer bar rectangle.
  const int max_speedometer_km_h = 100;
  const int full_speedometer_height =
      out_velocity.rows - digits_size.height - 3 * element_margin;
  const int speedometer_bar_horizontal_margin = 30;
  // Number of rows from below to fill corresponding to the current velocity.
  const int marked_speedometer_height =
      std::max(static_cast<int>(static_cast<double>(full_speedometer_height *
                                                    velocity_km_h) /
                                static_cast<double>(max_speedometer_km_h)),
               1);
  // Full speedometer bar outline.
  cv::rectangle(out_velocity,
                cv::Point(speedometer_bar_horizontal_margin, element_margin),
                cv::Point(out_velocity.cols - speedometer_bar_horizontal_margin,
                          element_margin + full_speedometer_height),
                text_color);
  // Filling in the velocity indication.
  cv::Mat marked_speedometer_bar =
      out_velocity
          .rowRange(element_margin + full_speedometer_height -
                        marked_speedometer_height,
                    element_margin + full_speedometer_height)
          .colRange(speedometer_bar_horizontal_margin,
                    out_velocity.cols - speedometer_bar_horizontal_margin);
  marked_speedometer_bar = text_color;
}

void MaybeRenderVelocity(const std::map<size_t, double> *velocities,
                         size_t frame_idx, cv::Mat *out_frame, int offset_rows,
                         int offset_cols, int window_rows, int window_cols) {
  if (velocities != nullptr) {
    const auto velocity_it = velocities->find(frame_idx);
    if (velocity_it != velocities->end()) {
      RenderVelocity(out_frame, offset_rows, offset_cols, window_rows,
                     window_cols, velocity_it->second);
    }
  }
}

cv::Mat LoadImageBgr2Rgb(const std::string &filename) {
  const cv::Mat image_bgr = cv::imread(filename, CV_LOAD_IMAGE_COLOR);
  CHECK(!image_bgr.empty());
  const cv::Mat image = image_bgr.clone();
  cv::cvtColor(image_bgr, image, cv::COLOR_BGR2RGB);
  return image;
}
}

int main(int argc, char **argv) {
  // Verify that the version of the library that we linked against is
  // compatible with the version of the headers we compiled against.
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();

  av_register_all();

  std::unique_ptr<pilotguru::ImageSequenceSource> image_source =
      pilotguru::MakeImageSequenceSource(FLAGS_in_video, FLAGS_vertical_flip,
                                         FLAGS_horizontal_flip);

  const cv::Mat steering_wheel = LoadImageBgr2Rgb(FLAGS_steering_wheel);

  std::unique_ptr<std::map<size_t, double>> steering_left =
      MaybeLoadSteering(FLAGS_steering_left_json, FLAGS_steering_left_units,
                        FLAGS_steering_left_scale);
  std::unique_ptr<std::map<size_t, double>> steering_right =
      MaybeLoadSteering(FLAGS_steering_right_json, FLAGS_steering_right_units,
                        FLAGS_steering_right_scale);
  constexpr double msToKmh = 3.6;
  std::unique_ptr<std::map<size_t, double>> velocities_left =
      MaybeLoadVelocities(FLAGS_velocities_json_left, msToKmh);
  std::unique_ptr<std::map<size_t, double>> velocities_right =
      MaybeLoadVelocities(FLAGS_velocities_json_right, msToKmh);

  pilotguru::ImageSequenceVideoFileSink sink(FLAGS_out_video, 30 /* fps */);

  std::unique_ptr<cv::Mat> out_frame(nullptr);
  int total_rendered_frames = 0;
  int skipped_frames = 0;
  //  pilotguru::RealTimeSeries::ValueLookupResult frame_velocity{0, false, 0};

  ORB_SLAM2::TimestampedImage frame;

  for (size_t frame_idx = 0; image_source->hasNext() &&
                             (total_rendered_frames < FLAGS_max_out_frames ||
                              FLAGS_max_out_frames < 0);
       ++frame_idx) {
    const ORB_SLAM2::TimestampedImage raw_frame = image_source->next();
    if (skipped_frames < FLAGS_frames_to_skip) {
      ++skipped_frames;
      continue;
    }

    const int32_t effective_video_height = FLAGS_target_video_height > 0
                                               ? FLAGS_target_video_height
                                               : raw_frame.image.rows;
    const int32_t effective_video_width = FLAGS_target_video_width > 0
                                              ? FLAGS_target_video_width
                                              : raw_frame.image.cols;

    if (out_frame == nullptr) {
      out_frame.reset(new cv::Mat(
          effective_video_height + steering_wheel.rows,
          std::max(effective_video_width, 4 * steering_wheel.cols), CV_8UC3));
      *out_frame = cv::Scalar_<unsigned char>(0, 0, 0);
    }

    cv::Mat out_video = out_frame->rowRange(0, effective_video_height)
                            .colRange(0, effective_video_width);
    cv::resize(raw_frame.image, out_video, out_video.size(), 0, 0,
               cv::INTER_CUBIC);

    // Fill frame bottom with black.
    out_frame->rowRange(frame.image.rows, out_frame->rows)
        .setTo(cv::Scalar(0, 0, 0));

    MaybeRenderSteering(steering_left.get(), frame_idx, out_frame.get(),
                        effective_video_height, 0, steering_wheel);
    MaybeRenderSteering(steering_right.get(), frame_idx, out_frame.get(),
                        effective_video_height,
                        out_frame->cols - steering_wheel.cols, steering_wheel);
    MaybeRenderVelocity(velocities_left.get(), frame_idx, out_frame.get(),
                        effective_video_height, steering_wheel.cols,
                        steering_wheel.rows, steering_wheel.cols);
    MaybeRenderVelocity(velocities_right.get(), frame_idx, out_frame.get(),
                        effective_video_height,
                        out_frame->cols - 2 * steering_wheel.cols,
                        steering_wheel.rows, steering_wheel.cols);

    sink.consume(*out_frame);

    ++total_rendered_frames;
  }

  return EXIT_SUCCESS;
}
