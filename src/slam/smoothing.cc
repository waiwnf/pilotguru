#include <slam/smoothing.hpp>

#include <opencv2/imgproc/imgproc.hpp>

namespace pilotguru {
// TODO: per-coordinate smoothing + normalization is a hack.
// Instead, it is better to do this properly as described in
// https://ntrs.nasa.gov/archive/nasa/casi.ntrs.nasa.gov/20070017872.pdf
void SmoothHeadingDirections(
    std::vector<ORB_SLAM2::PoseWithTimestamp> *trajectory, int sigma) {
  CHECK_NOTNULL(trajectory);
  CHECK_GT(sigma, 0);
  const cv::Mat smoothing_kernel = cv::getGaussianKernel(sigma * 4 + 1, sigma);
  const cv::Mat unit_kernel =
      cv::getGaussianKernel(1 /* size */, 1 /* sigma */);

  cv::Mat raw_rotations(4, trajectory->size(), CV_64F);
  for (size_t element_idx = 0; element_idx < trajectory->size();
       ++element_idx) {
    const Eigen::Quaterniond &rotation =
        trajectory->at(element_idx).pose.rotation;
    raw_rotations.at<double>(0, element_idx) = rotation.w();
    raw_rotations.at<double>(1, element_idx) = rotation.x();
    raw_rotations.at<double>(2, element_idx) = rotation.y();
    raw_rotations.at<double>(3, element_idx) = rotation.z();
  }
  cv::Mat smooth_rotations(4, trajectory->size(), CV_64F);
  cv::sepFilter2D(raw_rotations, smooth_rotations, CV_64F /* ddepth */,
                  smoothing_kernel /* kernelX */, unit_kernel /* kernelY */,
                  cv::Point(-1, -1) /* anchor */, 0 /* delta */,
                  cv::BORDER_REPLICATE);

  for (size_t element_idx = 0; element_idx < trajectory->size();
       ++element_idx) {
    const double element_norm =
        cv::norm(smooth_rotations.col(element_idx), cv::NORM_L2);

    Eigen::Quaterniond &rotation = trajectory->at(element_idx).pose.rotation;
    rotation.w() = smooth_rotations.at<double>(0, element_idx) / element_norm;
    rotation.x() = smooth_rotations.at<double>(1, element_idx) / element_norm;
    rotation.y() = smooth_rotations.at<double>(2, element_idx) / element_norm;
    rotation.z() = smooth_rotations.at<double>(3, element_idx) / element_norm;
  }
}
}