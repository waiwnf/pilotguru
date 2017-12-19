#ifndef PILOTGURU_MATH_MATH_HPP_
#define PILOTGURU_MATH_MATH_HPP_

namespace pilotguru {

// Summing accumulator using Kahan's algorithm:
// https://en.wikipedia.org/wiki/Kahan_summation_algorithm
template <typename T> class KahanSum {
public:
  KahanSum(const T &init_sum, const T &init_remainder)
      : sum_(init_sum), remainder_(init_remainder) {}

  void add(const T &v) {
    const T proposed_update = v + remainder_;
    const T updated_sum = sum_ + proposed_update;
    const T actual_update = updated_sum - sum_;
    remainder_ = proposed_update - actual_update;
    sum_ = updated_sum;
  }

  const T &sum() const { return sum_; }

private:
  T sum_, remainder_;
};
}

#endif // PILOTGURU_MATH_MATH_HPP_
