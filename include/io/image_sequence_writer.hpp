#ifndef PILOTGURU_IO_IMAGE_SEQUENCE_WRITER_HPP_
#define PILOTGURU_IO_IMAGE_SEQUENCE_WRITER_HPP_

#include <string>

#include <opencv2/highgui/highgui.hpp>

extern "C" {
#include <libavformat/avformat.h>
#include <libswscale/swscale.h>
}

namespace pilotguru {

class ImageSequenceSink {
public:
  virtual ~ImageSequenceSink() {}
  // Image must be RGB 8_UC3
  virtual void consume(const cv::Mat &image) = 0;
};

class ImageSequenceVideoFileSink : public ImageSequenceSink {
public:
  ImageSequenceVideoFileSink(const std::string &filename, int fps);
  virtual ~ImageSequenceVideoFileSink();

  void consume(const cv::Mat &image) override;

private:
  void InitStream(int height, int width);

  const std::string filename_;
  const int fps_;

  AVFrame *out_frame_ = nullptr;
  AVFrame *rgb_frame_ = nullptr;

  AVFormatContext *format_context_ = nullptr;
  AVStream *avstream_ = nullptr;
  AVCodec *codec_ = nullptr;
  AVCodecContext *codec_context_ = nullptr;
  int next_pts_ = 0;
  SwsContext *sws_context_ = nullptr;
};

} // namespace pilotguru

#endif /* PILOTGURU_IO_IMAGE_SEQUENCE_WRITER_HPP_ */
