#ifndef PILOTGURU_IO_IMAGE_SEQUENCE_READER_HPP_
#define PILOTGURU_IO_IMAGE_SEQUENCE_READER_HPP_

#include <string>
#include <vector>

#include <System.h>

#include <opencv2/highgui/highgui.hpp>

extern "C" {
#include <libavformat/avformat.h>
#include <libswscale/swscale.h>
}

namespace pilotguru {

struct TimestampedImageName {
  double timestamp = 0;
  std::string imageName;
};

class ImageSequenceSource {
public:
  virtual ~ImageSequenceSource() {}
  virtual bool hasNext() = 0;
  virtual ORB_SLAM2::TimestampedImage next() = 0;
};

class FlippedImageSequenceSource : public ImageSequenceSource {
public:
  // Takes ownership of the source.
  FlippedImageSequenceSource(std::unique_ptr<ImageSequenceSource> source,
                             int flip_axis);
  bool hasNext() override;
  ORB_SLAM2::TimestampedImage next() override;

private:
  std::unique_ptr<ImageSequenceSource> source_;
  const int flip_axis_;
};

class VideoImageSequenceSource : public ImageSequenceSource {
public:
  VideoImageSequenceSource(const std::string &filename);
  virtual ~VideoImageSequenceSource();
  bool hasNext() override;
  ORB_SLAM2::TimestampedImage next() override;

private:
  void fetchNext();

  AVFormatContext *format_context_ = nullptr;
  int video_stream_index_ = -1;
  AVCodecContext *codec_context_ = nullptr;
  AVCodec *codec_ = nullptr;
  cv::Mat raw_frame_image_;  // Before rotations.
  ORB_SLAM2::TimestampedImage next_frame_;
  bool has_next_ = false;
  AVFrame *source_frame_ = nullptr;
  AVFrame *rgb_frame_ = nullptr;
  SwsContext *sws_context_ = nullptr;
  // Counterclockwise.
  int rotate_degrees_ = 0;
};

std::unique_ptr<ImageSequenceSource>
MakeImageSequenceSource(const std::string &filename, bool vertical_flip,
                        bool horizontal_flip);

std::vector<TimestampedImageName> LoadImages(const std::string &strFile);

} // namespace pilotguru

#endif /* PILOTGURU_IO_IMAGE_SEQUENCE_READER_HPP_ */
