// Takes in a video and camera calibration parameters.
// Outputs camera trajectories (as JSON files), based on the results of
// ORB_SLAM2 SLAM logic, with additional preprocessing:
//
//   - PCA projection to find the dominant plane of motion. Assuming that the
//     camera was fixed on a vehicle driving on relatively flat terrain, this
//     will yield the roughly horizontal plane (these assumptions will fail for
//     driving on e.g. mountain switchbacks - terrain not flat/horizontal, or
//     when
//     driving only in a straight line - rotations of the real horizontal plane
//     around the line of motion will capture the trajectory almost as well as
//     the real horizontal plane.
//     We then project rotations on this horizontal plane to get a notion of
//     left/right turns. The inferred projected angular velocity can be used
//     as to train a machine learning model to provide automated steering.
//
//   - Gaussian smoothing (across time) of inferred camera orientations to
//     remove the high frequency noise from the SLAM system.

#include <iostream>
#include <memory>

#include <gflags/gflags.h>
#include <glog/logging.h>

extern "C" {
#include <libavformat/avformat.h>
#include <libswscale/swscale.h>
}

#include <System.h>

#include <io/image_sequence_reader.hpp>
#include <slam/track_image_sequence.hpp>

DEFINE_string(vocabulary_file, "", "ORB vocabulary file.");
DEFINE_string(camera_settings, "",
              ".yml file name to write the calibration parameters to.");
DEFINE_string(out_dir, "",
              "Directory to write trajectories and (optionally) "
              "corresponding subvideos of the segments to. There "
              "may be several trajectories (not overlapping in "
              "time) from a single video if the SLAM system loses "
              "tracking between frames and needs to be restarted.");
DEFINE_string(in_video, "", "Input video file.");
DEFINE_bool(
    visualize, true,
    "Whether to show the video and 3D map on screen during processing.");
DEFINE_bool(vertical_flip, false,
            "Whether to flip input video frames vertically.");
DEFINE_bool(horizontal_flip, false,
            "Whether to flip input video frames horizontally.");
DEFINE_bool(output_per_segment_videos, false,
            "Whether to write video files for every successfully SLAM-tracked "
            "trajectory segment. If this flag is true, the frame IDs in the "
            "output JSON files will correspond to frames in the segment video, "
            "not in the overall input video.");

namespace {
std::string TrajectoryOutFileName(const std::string &out_dir, int segment_id,
                                  const std::string &extension) {
  std::stringstream name_stream;
  name_stream << out_dir << "/trajectory-" << segment_id << "." << extension;
  return name_stream.str();
}
} // namespace

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();
  CHECK(!FLAGS_vocabulary_file.empty());
  CHECK(!FLAGS_camera_settings.empty());
  CHECK(!FLAGS_in_video.empty());

  av_register_all();

  std::unique_ptr<pilotguru::ImageSequenceSource> image_source =
      pilotguru::MakeImageSequenceSource(FLAGS_in_video, FLAGS_vertical_flip,
                                         FLAGS_horizontal_flip);

  std::unique_ptr<ORB_SLAM2::ORBVocabulary> vocabulary(
      new ORB_SLAM2::ORBVocabulary(FLAGS_vocabulary_file));

  // Outer loop to handle cases of losing tracking in the middle of the video.
  for (int segment_id = 0; image_source->hasNext(); ++segment_id) {
    ORB_SLAM2::System *SLAM =
        new ORB_SLAM2::System(vocabulary.get(), FLAGS_camera_settings,
                              ORB_SLAM2::System::MONOCULAR, FLAGS_visualize);
    const std::string trajectory_json_name =
        TrajectoryOutFileName(FLAGS_out_dir, segment_id, "json");
    std::unique_ptr<pilotguru::ImageSequenceSink> trajectory_video_sink(
        nullptr);
    if (FLAGS_output_per_segment_videos) {
      const std::string trajectory_video_name =
          TrajectoryOutFileName(FLAGS_out_dir, segment_id, "mp4");
      trajectory_video_sink.reset(new pilotguru::ImageSequenceVideoFileSink(
          trajectory_video_name, 30 /* fps */));
    }
    pilotguru::TrackImageSequence(SLAM, *image_source, trajectory_json_name,
                                  trajectory_video_sink.get());
    SLAM->Shutdown();

    delete SLAM;
  }

  return EXIT_SUCCESS;
}
