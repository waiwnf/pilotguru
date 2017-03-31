#ifndef PILOTGURU_SLAM_HORIZONTAL_FLATTEN_HPP_
#define PILOTGURU_SLAM_HORIZONTAL_FLATTEN_HPP_

#include <memory>
#include <vector>

#include <System.h>

#include <opencv2/core/core.hpp>

namespace pilotguru {
std::unique_ptr<std::vector<cv::Mat>>
ProjectDirections(const std::vector<ORB_SLAM2::PoseWithTimestamp> &trajectory,
                  const cv::Mat &projection_plane);

void ProjectTranslations(std::vector<ORB_SLAM2::PoseWithTimestamp> *trajectory,
                         const cv::Mat &projection_plane);

vector<double>
Projected2DDirectionsToTurnAngles(const std::vector<cv::Mat> &directions);
}

#endif // PILOTGURU_SLAM_HORIZONTAL_FLATTEN_HPP_