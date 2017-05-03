#include <optimization/gradient_descent.hpp>

#include <glog/logging.h>

namespace pilotguru {

GradientDescent::GradientDescent(double start_learning_rate,
                                 double learning_rate_decay,
                                 double min_gradient_clip,
                                 double max_gradient_clip)
    : start_learning_rate_(start_learning_rate),
      learning_rate_decay_(learning_rate_decay),
      min_gradient_clip_(min_gradient_clip),
      max_gradient_clip_(max_gradient_clip) {
  CHECK_LT(min_gradient_clip, max_gradient_clip);
}

void GradientDescent::optimize(LossFunction &loss,
                               std::vector<double> *parameters, size_t iters) {
  CHECK_NOTNULL(parameters);
  double learning_rate = start_learning_rate_;
  std::vector<double> gradient(parameters->size(), 0.0);
  for (size_t iter = 0; iter < iters; ++iter) {
    const double loss_value = loss.eval(*parameters, &gradient);
    LOG_EVERY_N(INFO, 10) << "Iter: " << iter << "  loss: " << loss_value;
    for (size_t i = 0; i < parameters->size(); ++i) {
      const double clipped_gradient = std::max(
          min_gradient_clip_, std::min(gradient.at(i), max_gradient_clip_));
      parameters->at(i) -= learning_rate * clipped_gradient;
    }
    learning_rate *= learning_rate_decay_;
  }
}

} // namespace pilotguru
