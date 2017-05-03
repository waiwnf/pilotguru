#ifndef PILOTGURU_OPTIMIZATION_GRADIENT_DESCENT_HPP_
#define PILOTGURU_OPTIMIZATION_GRADIENT_DESCENT_HPP_

#include <cfloat>

#include <optimization/optimizer.hpp>

namespace pilotguru {

class GradientDescent : public Optimizer {
public:
  GradientDescent(double start_learning_rate, double learning_rate_decay,
                  double min_gradient_clip = -DBL_MAX,
                  double max_gradient_clip = DBL_MAX);

  void optimize(LossFunction &loss, std::vector<double> *parameters,
                size_t iters) override;

private:
  const double start_learning_rate_, learning_rate_decay_;
  const double min_gradient_clip_, max_gradient_clip_;
};

} // namespace pilotguru

#endif // PILOTGURU_OPTIMIZATION_GRADIENT_DESCENT_HPP_