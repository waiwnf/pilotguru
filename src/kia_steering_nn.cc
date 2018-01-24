#include <iostream>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <zmq.hpp>

DEFINE_string(steering_prediction_socket, "ipc:///tmp/steering-predict", "");

int main(int argc, char *argv[]) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();

  zmq::context_t context(1);
  zmq::socket_t socket(context, ZMQ_SUB);
  int conflate_on = 1;
  socket.setsockopt(ZMQ_CONFLATE, &conflate_on, sizeof(int));
  socket.connect(FLAGS_steering_prediction_socket);
  // Subscribe to all the messages from the publisher.
  socket.setsockopt(ZMQ_SUBSCRIBE, "", 0);

  std::cout << "Connected" << std::endl;

  while (true) {
    zmq::message_t request;

    //  Wait for next request from client
    socket.recv(&request);
    const std::string request_text(static_cast<const char *>(request.data()),
                                   request.size());
    std::cout << "Received Hello " << request.size() << " " << request_text
              << std::endl;
  }

  return EXIT_SUCCESS;
}
