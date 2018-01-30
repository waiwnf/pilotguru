#ifndef NN_COMM_NN_COMM_HPP_
#define NN_COMM_NN_COMM_HPP_

#include <memory>
#include <mutex>
#include <thread>

#include <car/timestamped_history.hpp>

#include <zmq.hpp>

namespace pilotguru {

class SingleSteeringAnglePredictionUpdater {
public:
  SingleSteeringAnglePredictionUpdater(zmq::socket_t *socket,
                                       size_t history_length);

  const TimestampedHistory<double> &Predictions() const;
  void Start();
  void Stop();

private:
  void UpdateLoop();

  zmq::socket_t *socket_;
  TimestampedHistory<double> steering_angle_predictions_;

  bool must_run_ = false;
  std::unique_ptr<std::thread> update_thread_;
  std::mutex update_thread_status_mutex_;
};

} // namespace pilotguru

#endif // NN_COMM_NN_COMM_HPP_
