// Makes a copy of the original video with frame numbers added to every frame.
// To be used for checking which frames of the original video to exclude from
// the model training datasets.

#include <iostream>
#include <memory>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <opencv2/imgproc/imgproc.hpp>

#include <io/image_sequence_reader.hpp>
#include <io/image_sequence_writer.hpp>

DEFINE_string(in_video, "", "Input video file.");
DEFINE_string(out_video, "", "Output video file to write.");
DEFINE_int64(frames_to_skip, 0, "Number of initial trajectory frames to skip.");
DEFINE_int64(max_out_frames, -1, "If positive, the maximum number of frames to "
                                 "render for the output video. If the input "
                                 "trajectory has more frames, the remaining "
                                 "ones will be ignored.");

int main(int argc, char **argv) {
  // Verify that the version of the library that we linked against is
  // compatible with the version of the headers we compiled against.
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();

  av_register_all();

  std::unique_ptr<pilotguru::ImageSequenceSource> image_source =
      pilotguru::MakeImageSequenceSource(FLAGS_in_video, false, false);

  pilotguru::ImageSequenceVideoFileSink sink(FLAGS_out_video, 30 /* fps */);

  int total_rendered_frames = 0;
  int skipped_frames = 0;
  for (size_t frame_idx = 0; image_source->hasNext() &&
                             (total_rendered_frames < FLAGS_max_out_frames ||
                              FLAGS_max_out_frames < 0);
       ++frame_idx) {
    ORB_SLAM2::TimestampedImage frame = image_source->next();
    if (skipped_frames < FLAGS_frames_to_skip) {
      ++skipped_frames;
      continue;
    }

    std::stringstream frame_idx_text;
    frame_idx_text << frame_idx;
    cv::putText(frame.image, frame_idx_text.str(), cv::Point(10, 100),
                cv::FONT_HERSHEY_SIMPLEX, 3.0 /* text_scale */,
                cv::Scalar(255, 0, 0) /* color */, 3 /* line thicknes */);

    sink.consume(frame.image);

    ++total_rendered_frames;
  }

  return EXIT_SUCCESS;
}
