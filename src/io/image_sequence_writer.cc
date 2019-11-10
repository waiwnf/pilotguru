#include <io/image_sequence_writer.hpp>

#include <glog/logging.h>

namespace pilotguru {

ImageSequenceVideoFileSink::ImageSequenceVideoFileSink(
    const std::string &filename, int fps)
    : filename_(filename), fps_(fps) {}

ImageSequenceVideoFileSink::~ImageSequenceVideoFileSink() {
  if (out_frame_ != nullptr) {
    // Flush the video codec buffer.
    while(MaybeWriteFrame(nullptr)) {}

    av_write_trailer(format_context_);
    avcodec_close(avstream_->codec);
    av_frame_free(&out_frame_);
    av_frame_free(&rgb_frame_);
    sws_freeContext(sws_context_);
    avio_closep(&format_context_->pb);
    avformat_free_context(format_context_);
  }
}

void ImageSequenceVideoFileSink::InitStream(int height, int width) {
  CHECK(out_frame_ == nullptr);

  out_frame_ = CHECK_NOTNULL(av_frame_alloc());
  rgb_frame_ = CHECK_NOTNULL(av_frame_alloc());

  avformat_alloc_output_context2(&format_context_, NULL, NULL,
                                 filename_.c_str());
  CHECK_NOTNULL(format_context_);

  format_context_->oformat->video_codec = AV_CODEC_ID_H264;

  codec_ = CHECK_NOTNULL(
      avcodec_find_encoder(format_context_->oformat->video_codec));
  avstream_ = CHECK_NOTNULL(avformat_new_stream(format_context_, codec_));
  avstream_->id = format_context_->nb_streams - 1;

  codec_context_ = avstream_->codec;
  codec_context_->codec_id = format_context_->oformat->video_codec;
  codec_context_->bit_rate = 4000000;
  codec_context_->width = width;
  codec_context_->height = height;

  avstream_->time_base = (AVRational){1, fps_};
  codec_context_->time_base = avstream_->time_base;
  codec_context_->gop_size =
      12; /* emit one intra frame every twelve frames at most */
  codec_context_->pix_fmt = AV_PIX_FMT_YUV420P;

  /* Some formats want stream headers to be separate. */
  if (format_context_->oformat->flags & AVFMT_GLOBALHEADER)
    codec_context_->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;

  CHECK_GE(avcodec_open2(codec_context_, codec_, nullptr), 0);
  av_dump_format(format_context_, 0, filename_.c_str(), 1);
  CHECK_GE(avio_open(&format_context_->pb, filename_.c_str(), AVIO_FLAG_WRITE),
           0);
  CHECK_GE(avformat_write_header(format_context_, nullptr), 0);

  const int rgb_bytes = avpicture_get_size(AV_PIX_FMT_RGB24, width, height);
  uint8_t *rgb_buffer = (uint8_t *)av_malloc(rgb_bytes * sizeof(uint8_t));
  CHECK_GE(avpicture_fill((AVPicture *)rgb_frame_, rgb_buffer, AV_PIX_FMT_RGB24,
                          width, height),
           0);
  rgb_frame_->height = height;
  rgb_frame_->width = width;
  rgb_frame_->format = AV_PIX_FMT_RGB24;

  const int out_bytes =
      avpicture_get_size(codec_context_->pix_fmt, width, height);
  uint8_t *out_buffer = (uint8_t *)av_malloc(out_bytes * sizeof(uint8_t));
  CHECK_GE(avpicture_fill((AVPicture *)out_frame_, out_buffer,
                          codec_context_->pix_fmt, width, height),
           0);
  out_frame_->height = height;
  out_frame_->width = width;
  out_frame_->format = codec_context_->pix_fmt;

  sws_context_ = sws_getCachedContext(
      sws_context_, width, height, AV_PIX_FMT_RGB24, width, height,
      codec_context_->pix_fmt, SWS_BILINEAR, nullptr, nullptr, nullptr);
}

void ImageSequenceVideoFileSink::consume(const cv::Mat &image) {
  if (out_frame_ == nullptr) {
    InitStream(image.rows, image.cols);
  }

  CHECK_EQ(image.rows, rgb_frame_->height);
  CHECK_EQ(image.cols, rgb_frame_->width);
  for (int row = 0; row < image.rows; ++row) {
    for (int col = 0; col < image.cols; ++col) {
      const cv::Vec3b &rgb = image.at<cv::Vec3b>(row, col);
      unsigned char *dest =
          rgb_frame_->data[0] + row * rgb_frame_->linesize[0] + 3 * col;
      dest[0] = rgb[0];
      dest[1] = rgb[1];
      dest[2] = rgb[2];
    }
  }

  sws_scale(sws_context_, rgb_frame_->data, rgb_frame_->linesize, 0, image.rows,
            out_frame_->data, out_frame_->linesize);
  out_frame_->pts = next_pts_++;

  MaybeWriteFrame(out_frame_);
}

int ImageSequenceVideoFileSink::MaybeWriteFrame(AVFrame *out_frame) {
  AVPacket pkt = {0};
  av_init_packet(&pkt);
  int got_packet = 0;
  CHECK_EQ(
      avcodec_encode_video2(avstream_->codec, &pkt, out_frame, &got_packet), 0);
  if (got_packet) {
    av_packet_rescale_ts(&pkt, avstream_->codec->time_base,
                         avstream_->time_base);
    pkt.stream_index = avstream_->index;
    CHECK_GE(av_interleaved_write_frame(format_context_, &pkt), 0);
  }
  return got_packet;
}

} // namespace pilotguru
