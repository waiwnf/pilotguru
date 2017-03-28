#include "ORBVocabulary.h"

#include <glog/logging.h>

namespace ORB_SLAM2 {

ORBVocabulary::ORBVocabulary(const string& text_file) {
  CHECK(loadFromTextFile(text_file));
}

} //namespace ORB_SLAM
