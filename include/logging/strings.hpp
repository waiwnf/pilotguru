#ifndef PILOTGURU_LOGGING_STRINGS_HPP_
#define PILOTGURU_LOGGING_STRINGS_HPP_

#include <sstream>
#include <string>
#include <vector>

template <typename T>
std::ostream &operator<<(std::ostream &out, const std::vector<T> &v) {
  out << "{";
  for (auto it = v.begin(); it != v.end(); ++it) {
    out << *it;
    if (it + 1 != v.end()) {
      out << ", ";
    }
  }
  out << "}";

  return out;
}

#endif // PILOTGURU_LOGGING_STRINGS_HPP_