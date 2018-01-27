#ifndef PILOTGURU_IO_TIMESTAMPED_JSON_LOGGER_HPP_
#define PILOTGURU_IO_TIMESTAMPED_JSON_LOGGER_HPP_

#include <ctime>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <car/timestamped_history.hpp>

#include <glog/logging.h>

namespace pilotguru {
template <typename T> class JsonSteamWriter {
public:
  virtual void WriteAsJsonString(const T &t, std::ostream &file_stream) = 0;
};

template <typename T> class TimestampedJsonLogger {
public:
  TimestampedJsonLogger(const std::string &filename,
                        const std::string &root_element_name,
                        std::unique_ptr<JsonSteamWriter<T>> value_writer,
                        const TimestampedHistory<T> *elements_source)
      : value_writer_(std::move(value_writer)),
        json_ostream_(new std::ofstream(filename)),
        elements_source_(elements_source) {
    CHECK_NOTNULL(elements_source_);

    // Write json preamble.
    *json_ostream_ << "{\n";
    *json_ostream_ << "  \"" << root_element_name << "\" : [\n";

    logging_thread_.reset(
        new std::thread(&TimestampedJsonLogger<T>::Loop, this));
  }

  ~TimestampedJsonLogger() {
    // Make sure Stop() was called and closed the file.
    CHECK(json_ostream_ == nullptr);
  }

  void Stop() {
    std::unique_lock<std::mutex> lock(stop_mutex_);
    if (must_run_) {
      must_run_ = false;
      logging_thread_->join();

      *json_ostream_ << "\n]\n}\n";

      json_ostream_.reset(nullptr);
    }
  }

private:
  void Loop() {
    Timestamped<T> element = {{}, {0, 0}};
    constexpr int time_string_buffer_size = 64;
    char time_string_buffer[time_string_buffer_size];
    LoopWaitEffectiveTimeout loop_timeout({0 /* seconds */, 50000 /* usec */});
    while (must_run_) {
      timeval wait_timeout = loop_timeout.GetRemainingTimeout();
      const bool wait_result = elements_source_->wait_get_next(
          element.timestamp(), &wait_timeout, &element);
      loop_timeout.WaitFinished();
      if (wait_result) {
        // Make a string with the total number of microseconds in the timestamp.
        const timeval &timestamp = element.timestamp();
        const int time_string_written_size =
            snprintf(time_string_buffer, time_string_buffer_size, "%ld%06ld",
                     static_cast<long>(timestamp.tv_sec), timestamp.tv_usec);
        if (time_string_written_size >= time_string_buffer_size) {
          LOG(ERROR) << "Timestamp string did not fit into a buffer: "
                     << static_cast<long>(timestamp.tv_sec) << " "
                     << timestamp.tv_usec;
          continue;
        }

        if (first_element_already_written_) {
          // We are not the first element, write out comma separator after the
          // previous one.
          *json_ostream_ << ",\n";
        }
        first_element_already_written_ = true;
        *json_ostream_ << "    {\n";

        *json_ostream_ << "      \"time_usec\" : " << time_string_buffer
                       << ",\n";
        *json_ostream_ << "      ";
        value_writer_->WriteAsJsonString(element.data(), *json_ostream_);
        *json_ostream_ << "    }";
      }
    }
  }

  std::unique_ptr<JsonSteamWriter<T>> value_writer_;
  std::unique_ptr<std::ofstream> json_ostream_;
  const TimestampedHistory<T> *const elements_source_;

  bool must_run_ = true;
  bool first_element_already_written_ = false;

  std::unique_ptr<std::thread> logging_thread_;
  // Guards Stop() to make sure we do not try to close the file twice.
  std::mutex stop_mutex_;
};

} // namespace pilotguru

#endif // PILOTGURU_IO_TIMESTAMPED_JSON_LOGGER_HPP_
