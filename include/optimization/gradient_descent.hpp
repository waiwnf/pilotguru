#ifndef PILOTGURU_OPTIMIZATION_GRADIENT_DESCENT_HPP_
#define PILOTGURU_OPTIMIZATION_GRADIENT_DESCENT_HPP_

#include <optimization/optimizer.hpp>

namespace pilotguru {

class GradientDescent : public Optimizer {
public:
  GradientDescent(double start_learning_rate, double learning_rate_decay);

  void optimize(LossFunction &loss, std::vector<double> *parameters,
                size_t iters) override;

private:
  const double start_learning_rate_, learning_rate_decay_;
};

} // namespace pilotguru

#endif // PILOTGURU_OPTIMIZATION_GRADIENT_DESCENT_HPP_