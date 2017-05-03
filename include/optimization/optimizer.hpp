#ifndef PILOTGURU_OPTIMIZATION_OPTIMIZER_HPP_
#define PILOTGURU_OPTIMIZATION_OPTIMIZER_HPP_

#include <cstddef>
#include <vector>

#include <optimization/loss_function.hpp>

namespace pilotguru {

class Optimizer {
public:
  virtual ~Optimizer() {}
  virtual void optimize(LossFunction &loss,
                        std::vector<double> *parameters, size_t iters) = 0;
};

} // namespace pilotguru

#endif // PILOTGURU_OPTIMIZATION_OPTIMIZER_HPP_