#ifndef PILOTGURU_SLAM_TRACK_IMAGE_SEQUENCE_HPP_
#define PILOTGURU_SLAM_TRACK_IMAGE_SEQUENCE_HPP_

#include <string>
#include <vector>

#include <System.h>

#include <io/image_sequence_reader.hpp>
#include <io/image_sequence_writer.hpp>

namespace pilotguru {

bool TrackImageSequence(ORB_SLAM2::System *SLAM,
                        ImageSequenceSource &image_source,
                        const std::string &trajectoryOutFile,
                        ImageSequenceSink *tracked_frames_sink);

} // namespace pilotguru

#endif /* PILOTGURU_SLAM_TRACK_IMAGE_SEQUENCE_HPP_ */
