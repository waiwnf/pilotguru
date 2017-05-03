#ifndef PILOTGURU_OPTIMIZATION_OBJECTIVE_HPP_
#define PILOTGURU_OPTIMIZATION_OBJECTIVE_HPP_

#include <vector>

namespace pilotguru {

class LossFunction {
public:
  virtual ~LossFunction() {}

  virtual double eval(const std::vector<double>& in, std::vector<double>* gradient) = 0;
};

} // namespace pilotguru

#endif // PILOTGURU_OPTIMIZATION_OBJECTIVE_HPP_