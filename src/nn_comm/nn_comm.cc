#include <nn_comm/nn_comm.hpp>

#include <glog/logging.h>
#include <json.hpp>

namespace pilotguru {

SingleSteeringAnglePredictionUpdater::SingleSteeringAnglePredictionUpdater(
    zmq::socket_t *socket, size_t history_length)
    : socket_(socket), steering_angle_predictions_(history_length) {
  CHECK_NOTNULL(socket_);
}

const TimestampedHistory<SingleSteeringAnglePrediction> &
SingleSteeringAnglePredictionUpdater::predictions() const {
  return steering_angle_predictions_;
}

void SingleSteeringAnglePredictionUpdater::start() {
  std::unique_lock<std::mutex> lock(update_thread_status_mutex_);
  if (update_thread_ == nullptr) {
    must_run_ = true;
    update_thread_.reset(new std::thread(
        &SingleSteeringAnglePredictionUpdater::updateLoop, this));
  }
}

void SingleSteeringAnglePredictionUpdater::stop() {
  std::unique_lock<std::mutex> lock(update_thread_status_mutex_);
  if (update_thread_ != nullptr) {
    must_run_ = false;
    update_thread_->join();
    update_thread_.reset(nullptr);
  }
}

void SingleSteeringAnglePredictionUpdater::updateLoop() {
  timeval message_time;
  std::unique_ptr<std::string> steering_raw_text;
  while (must_run_) {
    {
      zmq::message_t steering_raw_message;
      //  Wait for next request from client
      const bool recv_success = socket_->recv(&steering_raw_message);
      if (!recv_success) {
        continue;
      }
      gettimeofday(&message_time, nullptr);
      steering_raw_text.reset(new std::string(
          static_cast<const char *>(steering_raw_message.data()),
          steering_raw_message.size()));
    }
    nlohmann::json steering_json = nlohmann::json::parse(*steering_raw_text);
    const double steering_degrees = steering_json["s"];
    steering_angle_predictions_.update({steering_degrees}, message_time);
  }
}

} // namespace pilotguru
