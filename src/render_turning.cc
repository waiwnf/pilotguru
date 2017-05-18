// From a video file and a JSON file with veocity and angular velocity on the
// horizontal plane produced by fit_motion.cc, generates a video tiled with a
// steering wheel picture rotating according to the horizontal angular velocity
// and a digital speedometer field.

#include <fstream>
#include <iostream>
#include <memory>

#include <json.hpp>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <opencv2/imgproc/imgproc.hpp>

#include <interpolation/time_series.hpp>
#include <io/image_sequence_reader.hpp>
#include <io/image_sequence_writer.hpp>
#include <io/json_converters.hpp>

DEFINE_string(in_video, "", "Input video file.");
DEFINE_bool(vertical_flip, false,
            "Whether to flip the input frames vertically.");
DEFINE_bool(horizontal_flip, false,
            "Whether to flip input video frames horizontally.");
DEFINE_string(frames_json, "", "JSON file with video frames timestamps. Comes "
                               "from the raw PilotGuru Recorder data.");
DEFINE_string(velocities_json, "", "JSON file with timestamped absolute "
                                   "velocities. Comes from fit_motion output.");
DEFINE_string(rotations_json, "", "JSON file with timestamped angular "
                                  "velocities in the horizontal plane (i.e. "
                                  "steering). Comes from fit_motion output.");
DEFINE_double(rotations_scale, 200.0,
              "Scale factor to go from angular velocity in radians (from the "
              "--rotations_json file) to the rotation angle of the steering "
              "wheel (in degrees) for visualization.");
DEFINE_double(rotations_smoothing_sigma, 0.1,
              "Steering angular velocity smoothing sigma, in seconds. If "
              "positive, steering angular velocity is smoothed out with this "
              "sigma before rendering.");
DEFINE_string(steering_wheel, "",
              "A file with the steering wheel image to use.");
DEFINE_string(out_video, "", "Output video file to write.");
DEFINE_int64(frames_to_skip, 0, "Number of initial trajectory frames to skip.");
DEFINE_int64(max_out_frames, -1, "If positive, the maximum number of frames to "
                                 "render for the output video. If the input "
                                 "trajectory has more frames, the remaining "
                                 "ones will be ignored.");

namespace {
void RenderRotation(cv::Mat *out_frame, int offset_rows, int offset_cols,
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

  CHECK(!FLAGS_frames_json.empty());
  std::unique_ptr<nlohmann::json> frames_json =
      pilotguru::ReadJsonFile(FLAGS_frames_json);
  const nlohmann::json &frames = (*frames_json)[pilotguru::kFrames];

  std::unique_ptr<pilotguru::RealTimeSeries> steering(nullptr);
  if (!FLAGS_rotations_json.empty()) {
    steering.reset(new pilotguru::RealTimeSeries(FLAGS_rotations_json,
                                                 pilotguru::kSteering,
                                                 pilotguru::kAngularVelocity));
    if (FLAGS_rotations_smoothing_sigma > 0) {
      steering->GaussianSmooth(FLAGS_rotations_smoothing_sigma);
    }
  }

  std::unique_ptr<pilotguru::RealTimeSeries> velocities(
      FLAGS_velocities_json.empty() ? nullptr : new pilotguru::RealTimeSeries(
                                                    FLAGS_velocities_json,
                                                    pilotguru::kVelocities,
                                                    pilotguru::kSpeedMS));

  pilotguru::ImageSequenceVideoFileSink sink(FLAGS_out_video, 30 /* fps */);

  std::unique_ptr<cv::Mat> out_frame(nullptr);
  int total_rendered_frames = 0;
  int skipped_frames = 0;
  pilotguru::RealTimeSeries::ValueLookupResult frame_velocity{0, false, 0};
  pilotguru::RealTimeSeries::ValueLookupResult frame_steering{0, false, 0};
  for (size_t frame_idx = 0; image_source->hasNext() &&
                             (total_rendered_frames < FLAGS_max_out_frames ||
                              FLAGS_max_out_frames < 0);
       ++frame_idx) {
    const ORB_SLAM2::TimestampedImage frame = image_source->next();
    if (skipped_frames < FLAGS_frames_to_skip) {
      ++skipped_frames;
      continue;
    }
    // We need interframe time difference to have a nontrivia;l interval for
    // averaging velocity and steering angle over, so skip the first frame.
    if (frame_idx == 0) {
      continue;
    }

    const long frame_usec = frames.at(frame_idx)[pilotguru::kTimeUsec];
    const long prev_frame_usec = frames.at(frame_idx - 1)[pilotguru::kTimeUsec];
    if (steering != nullptr) {
      frame_steering = steering->TimeAveragedValue(prev_frame_usec, frame_usec,
                                                   frame_steering.end_index);
    }

    if (velocities != nullptr) {
      frame_velocity = velocities->TimeAveragedValue(
          prev_frame_usec, frame_usec, frame_velocity.end_index);
    }

    if (!frame_steering.is_valid && !frame_velocity.is_valid) {
      // The video frame is out of bounds for the velocity/steering time series.
      continue;
    }

    if (out_frame == nullptr) {
      out_frame.reset(new cv::Mat(
          frame.image.rows + steering_wheel.rows,
          std::max(frame.image.cols, steering_wheel.cols), CV_8UC3));
      *out_frame = cv::Scalar_<unsigned char>(0, 0, 0);
    }

    // Double check the sizes match in case the out_frame was initialized on an
    // earlier iteration.
    CHECK_EQ(out_frame->rows, frame.image.rows + steering_wheel.rows);
    CHECK_EQ(out_frame->cols, std::max(frame.image.cols, steering_wheel.cols));
    cv::Mat out_video =
        out_frame->rowRange(0, frame.image.rows).colRange(0, frame.image.cols);
    frame.image.copyTo(out_video);

    if (frame_steering.is_valid) {
      RenderRotation(out_frame.get(), frame.image.rows, 0, steering_wheel,
                     frame_steering.value * FLAGS_rotations_scale);
    }

    if (frame_velocity.is_valid) {
      RenderVelocity(out_frame.get(), frame.image.rows, steering_wheel.cols,
                     steering_wheel.rows, steering_wheel.cols,
                     static_cast<int>(frame_velocity.value * 3.6));
    }

    sink.consume(*out_frame);

    ++total_rendered_frames;
  }

  return EXIT_SUCCESS;
}
