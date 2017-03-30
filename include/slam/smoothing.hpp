#ifndef PILOTGURU_SLAM_SMOOTHING_HPP_
#define PILOTGURU_SLAM_SMOOTHING_HPP_

#include <vector>

#include <System.h>

namespace pilotguru {

// Smooth camera heading directions with Gaussina kernel weights over nearby
// frames.
// sigma is in units of inter-frame distances.
void SmoothHeadingDirections(
    std::vector<ORB_SLAM2::PoseWithTimestamp> *trajectory, int sigma);

} // namespace pilotguru

#endif // #ifndef PILOTGURU_SLAM_SMOOTHING_HPP_