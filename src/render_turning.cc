// From a video file and a JSON file with a 3D trajectory produced by
// optical_trajectories.cc, generates a video tiled with a steering wheel
// picture that rotates according to the horizontal angular velocity from the
// JSON file.

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
DEFINE_string(trajectory_json, "",
              "JSON file with the trajectory for (a subsequence of) the input "
              "video. See optical_trajectories.cc to generate the trajectory "
              "from video automatically using SLAM.");
DEFINE_double(scale, 6000.0, "Scale factor to go from angular velocity in "
                             "radians (from the 3D trajectory in the JSON "
                             "file) to the rotation angle of the steering "
                             "wheel (in degrees) for visualization.");
DEFINE_double(learning_rate, 1.0,
              "Temporal smoothing parameter for the rendered steering wheel. "
              "Must be between 0 and 1. ! corresponds to no smoothing at all, "
              "0 corresponds to no change. The formula is "
              "steering_wheel_angle_[next frame] = learning_rate * scale * "
              "angular_velocity_from_json + (1 - learning_rate) * "
              "steering_wheel_angle_[prev frame].");
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

  const cv::Mat steering_wheel_bgr =
      cv::imread(FLAGS_steering_wheel, CV_LOAD_IMAGE_COLOR);
  CHECK(!steering_wheel_bgr.empty());
  const cv::Mat steering_wheel = steering_wheel_bgr.clone();
  cv::cvtColor(steering_wheel_bgr, steering_wheel, cv::COLOR_BGR2RGB);

  std::unique_ptr<nlohmann::json> trajectory_json(nullptr);
  nlohmann::json *trajectory = nullptr;
  if (!FLAGS_trajectory_json.empty()) {
    trajectory_json = pilotguru::ReadJsonFile(FLAGS_trajectory_json);
    trajectory = &(*trajectory_json)[pilotguru::kTrajectory];
  }

  pilotguru::ImageSequenceVideoFileSink sink(FLAGS_out_video, 30 /* fps */);

  std::unique_ptr<cv::Mat> out_frame(nullptr);
  double turn = 0;
  int total_rendered_frames = 0;
  int skipped_frames = 0;
  size_t trajectory_idx = 0;
  while (image_source->hasNext() &&
         (total_rendered_frames < FLAGS_max_out_frames ||
          FLAGS_max_out_frames < 0)) {
    const ORB_SLAM2::TimestampedImage frame = image_source->next();
    const bool render_steering =
        (trajectory != nullptr) && trajectory_idx < trajectory->size() &&
        trajectory->at(trajectory_idx)[pilotguru::kFrameId] <= frame.frame_id;
    if (!render_steering) {
      // The video frame is earlier than the current trajectory point. Advance
      // the frame, but not the trajectory iterator, to get the frames to catch
      // up.
      continue;
    }

    if (out_frame == nullptr) {
      out_frame.reset(new cv::Mat(
          frame.image.rows + steering_wheel.rows,
          std::max(frame.image.cols, steering_wheel.cols), CV_8UC3));
    }
    // Double check the sizes match in case the out_frame was initialized on an
    // earlier iteration.
    CHECK_EQ(out_frame->rows, frame.image.rows + steering_wheel.rows);
    CHECK_EQ(out_frame->cols, std::max(frame.image.cols, steering_wheel.cols));
    cv::Mat out_video =
        out_frame->rowRange(0, frame.image.rows).colRange(0, frame.image.cols);
    frame.image.copyTo(out_video);

    // Read off the raw turn angle and advance the turn angles index.
    double raw_turn = 0;
    if (render_steering) {
      // Read off the raw value
      const auto &point = trajectory->at(trajectory_idx);
      CHECK_EQ(frame.frame_id, point[pilotguru::kFrameId]);
      raw_turn = point[pilotguru::kTurnAngle];
      ++trajectory_idx;
    }

    // This goes after advancing all the annotations iterators to make sure
    // we skip annotations also along with the raw frames.
    if (skipped_frames < FLAGS_frames_to_skip) {
      ++skipped_frames;
      continue;
    }

    if (render_steering) {
      turn =
          (1.0 - FLAGS_learning_rate) * turn + FLAGS_learning_rate * raw_turn;
      RenderRotation(out_frame.get(), frame.image.rows, 0, steering_wheel,
                     turn * FLAGS_scale);
    }

    sink.consume(*out_frame);

    ++total_rendered_frames;
  }

  return EXIT_SUCCESS;
}
