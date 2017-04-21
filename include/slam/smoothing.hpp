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

std::vector<double> SmoothTimeSeries(
    const std::vector<double> &data_values,
    const std::vector<double>
        &data_timestamps /* corresponding to data_values, must be ordered */,
    const std::vector<double> &target_timestamps /* must be ordered */,
    double sigma /* in units of time */);

} // namespace pilotguru

#endif // #ifndef PILOTGURU_SLAM_SMOOTHING_HPP_