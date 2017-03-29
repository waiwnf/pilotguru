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
#include <io/json_constants.hpp>

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
DEFINE_double(learning_rate, 0.2,
              "Temporal smoothing parameter for the rendered steering wheel. "
              "Must be between 0 and 1. ! corresponds to no smoothing at all, "
              "0 corresponds to no change. The formula is "
              "steering_wheel_angle_[next frame] = learning_rate * scale * "
              "angular_velocity_from_json + (1 - learning_rate) * "
              "steering_wheel_angle_[prev frame].");
DEFINE_string(steering_wheel, "",
              "A file with the steering wheel image to use.");
DEFINE_string(out_video, "", "Output video file to write.");

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

  std::ifstream trajectory_file(FLAGS_trajectory_json);
  nlohmann::json trajectory_json;
  trajectory_file >> trajectory_json;
  const auto &trajectory = trajectory_json[pilotguru::kTrajectory];

  pilotguru::ImageSequenceVideoFileSink sink(FLAGS_out_video, 30 /* fps */);

  std::unique_ptr<cv::Mat> out_frame(nullptr);
  double turn = 0;
  for (auto trajectory_it = trajectory.begin();
       trajectory_it != trajectory.end() && image_source->hasNext();) {
    const ORB_SLAM2::TimestampedImage frame = image_source->next();
    const int64 frame_id = (*trajectory_it)[pilotguru::kFrameId];
    if (frame.frame_id < frame_id) {
      // The frame is earlier than the current trajectory point. Advance the
      // frame, but not the trajectory iterator, to get the frames to catch up.
      continue;
    }
    CHECK_EQ(frame.frame_id, frame_id);
    const double raw_turn = (*trajectory_it)[pilotguru::kTurnAngle];
    turn = (1.0 - FLAGS_learning_rate) * turn + FLAGS_learning_rate * raw_turn;
    LOG(INFO) << "Frame: " << frame_id << " turn: " << turn;

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
    cv::Mat out_steer =
        out_frame
            ->rowRange(frame.image.rows, frame.image.rows + steering_wheel.rows)
            .colRange(0, steering_wheel.cols);

    LOG(INFO) << "Frame: " << frame_id << " turn: " << turn;
    frame.image.copyTo(out_video);

    cv::Mat rotation_matrix = cv::getRotationMatrix2D(
        cv::Point2f(steering_wheel.cols / 2, steering_wheel.rows / 2),
        turn * FLAGS_scale, 1);
    cv::warpAffine(steering_wheel, out_steer, rotation_matrix,
                   steering_wheel.size(), cv::INTER_LINEAR);
    sink.consume(*out_frame);

    ++trajectory_it;
  }

  return EXIT_SUCCESS;
}
