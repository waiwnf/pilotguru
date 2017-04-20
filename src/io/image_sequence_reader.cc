#include <io/image_sequence_reader.hpp>

#include <fstream>
#include <iostream>

#include <glog/logging.h>

namespace pilotguru {

namespace {
void GetLineOrDie(std::istream *is, std::string *str) {
  CHECK_NOTNULL(is);
  CHECK_NOTNULL(str);
  CHECK(!is->eof());
  getline(*is, *str);
}
} // namespace

std::vector<TimestampedImageName> LoadImages(const std::string &strFile) {
  std::ifstream f;
  f.open(strFile.c_str());
  CHECK(!f.fail());

  // skip first three lines
  std::string line;
  GetLineOrDie(&f, &line);
  GetLineOrDie(&f, &line);
  GetLineOrDie(&f, &line);

  std::vector<TimestampedImageName> result;
  while (!f.eof()) {
    GetLineOrDie(&f, &line);
    std::cout << "Image: " << line;
    if (!line.empty()) {
      std::stringstream lineStream(line);
      TimestampedImageName imageData;
      lineStream >> imageData.timestamp;

      CHECK(!lineStream.eof());
      lineStream >> imageData.imageName;

      result.push_back(imageData);
    }
  }
  return result;
}

FlippedImageSequenceSource::FlippedImageSequenceSource(
    std::unique_ptr<ImageSequenceSource> source, int flip_axis)
    : source_(std::move(source)), flip_axis_(flip_axis) {}

bool FlippedImageSequenceSource::hasNext() { return source_->hasNext(); }

ORB_SLAM2::TimestampedImage FlippedImageSequenceSource::next() {
  ORB_SLAM2::TimestampedImage result = source_->next();
  cv::Mat flipped_image = result.image.clone();
  cv::flip(result.image, flipped_image, flip_axis_);
  result.image = flipped_image;
  return result;
}

namespace {
int VideoStreamIndexOrDie(const AVFormatContext &format_context) {
  for (unsigned int i = 0; i < format_context.nb_streams; ++i) {
    if (format_context.streams[i]->codec->codec_type == AVMEDIA_TYPE_VIDEO) {
      return i;
    }
  }
  CHECK(false) << "Inspected all the streams, but no video stream found.";
  return -1;
}
}

VideoImageSequenceSource::VideoImageSequenceSource(const std::string &filename)
    : format_context_(avformat_alloc_context()),
      source_frame_(av_frame_alloc()), rgb_frame_(av_frame_alloc()) {

  CHECK_EQ(avformat_open_input(&format_context_, filename.c_str(), 0, nullptr),
           0);
  CHECK_GE(avformat_find_stream_info(format_context_, nullptr), 0);

  av_dump_format(format_context_, 0, filename.c_str(), 0);

  video_stream_index_ = VideoStreamIndexOrDie(*format_context_);
  CHECK_GE(video_stream_index_, 0);

  codec_context_ =
      CHECK_NOTNULL(format_context_->streams[video_stream_index_]->codec);
  codec_ = CHECK_NOTNULL(avcodec_find_decoder(codec_context_->codec_id));
  CHECK_GE(avcodec_open2(codec_context_, codec_, nullptr), 0);

  const int rgb_bytes = avpicture_get_size(
      AV_PIX_FMT_RGB24, codec_context_->width, codec_context_->height);
  uint8_t *buffer = (uint8_t *)av_malloc(rgb_bytes * sizeof(uint8_t));
  avpicture_fill((AVPicture *)rgb_frame_, buffer, AV_PIX_FMT_RGB24,
                 codec_context_->width, codec_context_->height);

  raw_frame_image_ =
      cv::Mat(codec_context_->height, codec_context_->width, CV_8UC3);

  sws_context_ = sws_getCachedContext(
      sws_context_, codec_context_->width, codec_context_->height,
      codec_context_->pix_fmt, codec_context_->width, codec_context_->height,
      AV_PIX_FMT_RGB24, SWS_BILINEAR, nullptr, nullptr, nullptr);

  // Read the requested video rotation.
  const AVDictionary *video_metadata =
      format_context_->streams[video_stream_index_]->metadata;
  const AVDictionaryEntry *rotate_entry =
      av_dict_get(video_metadata, "rotate", nullptr, 0 /* flags */);
  if (rotate_entry != nullptr) {
    CHECK(sscanf(rotate_entry->value, "%d", &rotate_degrees_));
  }
  rotate_degrees_ %= 360;

  fetchNext();
}

VideoImageSequenceSource::~VideoImageSequenceSource() {
  av_free(rgb_frame_);
  av_free(source_frame_);
  avcodec_close(codec_context_);
  avformat_close_input(&format_context_);
}

bool VideoImageSequenceSource::hasNext() { return has_next_; }

ORB_SLAM2::TimestampedImage VideoImageSequenceSource::next() {
  CHECK(has_next_);
  ORB_SLAM2::TimestampedImage result;
  next_frame_.image.copyTo(result.image);
  result.timestamp = next_frame_.timestamp;
  result.frame_id = next_frame_.frame_id;
  fetchNext();
  return result;
}

void VideoImageSequenceSource::fetchNext() {
  has_next_ = false;
  AVPacket packet;
  while (av_read_frame(format_context_, &packet) >= 0) {
    if (packet.stream_index != video_stream_index_) {
      continue;
    }

    int got_picture = 0;
    avcodec_decode_video2(codec_context_, source_frame_, &got_picture, &packet);
    if (!got_picture) {
      continue;
    }
    sws_scale(sws_context_, source_frame_->data, source_frame_->linesize, 0,
              source_frame_->height, rgb_frame_->data, rgb_frame_->linesize);

    for (int row = 0; row < source_frame_->height; ++row) {
      for (int col = 0; col < source_frame_->width; ++col) {
        cv::Vec3b &dest = raw_frame_image_.at<cv::Vec3b>(row, col);
        const unsigned char *src =
            rgb_frame_->data[0] + row * rgb_frame_->linesize[0] + 3 * col;
        dest[0] = src[0];
        dest[1] = src[1];
        dest[2] = src[2];
      }
    }

    next_frame_.timestamp =
        av_frame_get_best_effort_timestamp(source_frame_) *
        av_q2d(format_context_->streams[video_stream_index_]->time_base);
    ++next_frame_.frame_id;
    has_next_ = true;
    break;
  }

  // http://stackoverflow.com/questions/16265673/rotate-image-by-90-180-or-270-degrees
  switch (rotate_degrees_) {
  case 0:
    raw_frame_image_.copyTo(next_frame_.image);
    break;
  case 90:
    cv::flip(raw_frame_image_.t(), next_frame_.image, 0);
    break;
  case 180:
    // 180 degrees rotations corresponds to flipping around both x and y axes.
    cv::flip(raw_frame_image_, next_frame_.image, -1);
    break;
  case 270:
    cv::flip(raw_frame_image_.t(), next_frame_.image, 1);
    break;
  default:
    LOG(FATAL) << "Unsupported rotation angle in video metadata: "
               << rotate_degrees_
               << ". Only multiples of 90 degrees rotations are supported.";
  }
}

namespace {
// See cv2::flip for flip axis interpretation.
int MakeFlipAxis(bool vertical_flip, bool horizontal_flip) {
  if (vertical_flip && !horizontal_flip) {
    return 0;
  } else if (!vertical_flip && horizontal_flip) {
    return 1;
  } else if (vertical_flip && horizontal_flip) {
    return -1;
  } else {
    CHECK(false)
        << "At least one of (vertical flip, horizontal flip) must be true.";
    return -1;
  }
}
} // namespace

std::unique_ptr<ImageSequenceSource>
MakeImageSequenceSource(const std::string &filename, bool vertical_flip,
                        bool horizontal_flip) {
  std::unique_ptr<ImageSequenceSource> file_source(
      new VideoImageSequenceSource(filename));
  if (!vertical_flip && !horizontal_flip) {
    return std::move(file_source);
  }

  const int flip_axis = MakeFlipAxis(vertical_flip, horizontal_flip);
  std::unique_ptr<ImageSequenceSource> flipped_source(
      new FlippedImageSequenceSource(std::move(file_source),
                                                flip_axis));
  return std::move(flipped_source);
}

} // namespace pilotguru
