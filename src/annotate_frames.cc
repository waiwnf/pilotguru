// Annotate timestamped frames with values from a JSON time series. Writes the
// results to a JSON file with [frame id] - [value] pairs.
// Effective value for a frame is the average time-weighted value of the time
// series on the interval between the previous frame and the annotated frame.

#include <fstream>
#include <iostream>
#include <memory>

#include <json.hpp>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <interpolation/time_series.hpp>
#include <io/json_converters.hpp>

DEFINE_string(frames_json, "", "JSON file with video frames timestamps. Comes "
                               "from the raw PilotGuru Recorder data.");
DEFINE_string(in_json, "", "JSON file with the source time series data.");
DEFINE_string(json_root_element_name, "",
              "Root JSON element, pointing to the time series list.");
DEFINE_string(json_value_name, "",
              "Value element name in the time series JSON file.");
DEFINE_string(out_json, "", "Filename to write to.");
DEFINE_double(smoothing_sigma, -1.0, "If positive, apply Gaussian smoothing "
                                     "with this sigma (in seconds) to the raw "
                                     "timeseries before computing frame "
                                     "annotations.");

int main(int argc, char **argv) {
  // Verify that the version of the library that we linked against is
  // compatible with the version of the headers we compiled against.
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();

  CHECK(!FLAGS_frames_json.empty());
  std::unique_ptr<nlohmann::json> frames_json =
      pilotguru::ReadJsonFile(FLAGS_frames_json);
  const nlohmann::json &frames = (*frames_json)[pilotguru::kFrames];

  CHECK(!FLAGS_in_json.empty());
  CHECK(!FLAGS_json_root_element_name.empty());
  CHECK(!FLAGS_json_value_name.empty());
  std::unique_ptr<pilotguru::RealTimeSeries> in_series(
      new pilotguru::RealTimeSeries(FLAGS_in_json, FLAGS_json_root_element_name,
                                    FLAGS_json_value_name));
  if (FLAGS_smoothing_sigma > 0) {
    in_series->GaussianSmooth(FLAGS_smoothing_sigma);
  }

  nlohmann::json out_json;
  out_json[FLAGS_json_root_element_name] = {};
  nlohmann::json &out_list = out_json[FLAGS_json_root_element_name];
  pilotguru::RealTimeSeries::ValueLookupResult frame_annotation{0, false, 0};
  for (size_t frame_idx = 1; frame_idx < frames.size(); ++frame_idx) {
    const auto &frame = frames.at(frame_idx);
    const long frame_usec = frame[pilotguru::kTimeUsec];
    const long prev_frame_usec = frames.at(frame_idx - 1)[pilotguru::kTimeUsec];
    frame_annotation = in_series->TimeAveragedValue(prev_frame_usec, frame_usec,
                                                    frame_annotation.end_index);
    if (frame_annotation.is_valid) {
      nlohmann::json out_event;
      out_event[pilotguru::kFrameId] = frame[pilotguru::kFrameId];
      out_event[FLAGS_json_value_name] = frame_annotation.value;
      out_list.push_back(out_event);
    }
  }
  std::ofstream json_ostream(FLAGS_out_json);
  json_ostream << out_json.dump(2) << std::endl;

  return EXIT_SUCCESS;
}
