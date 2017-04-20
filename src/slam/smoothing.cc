#include <slam/smoothing.hpp>

#include <cmath>

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

namespace {
double NormalCdf(double x, double mean, double sigma) {
  CHECK_GT(sigma, 0);
  static const double sqrt_2 = sqrt(2.0);
  return 0.5 * (1.0 + erf((x - mean) / (sqrt_2 * sigma)));
}
} // namespace

vector<double> SmoothTimeSeries(const vector<double> &timestamps,
                                const vector<double> &values, double sigma) {
  CHECK_GT(sigma, 0);
  CHECK_EQ(timestamps.size(), values.size());
  vector<double> result(values.size(), 0);

  size_t left_idx = 0;  // Left boundary of the smoothing window.
  size_t right_idx = 0; // Right boundary of the smoothing window.
  for (size_t target_idx = 0; target_idx < values.size(); ++target_idx) {
    // Move the smoothing window boundaries to just outside 3 sigma away from
    // the target timestamp.
    while (left_idx + 1 < values.size() &&
           timestamps.at(target_idx) - timestamps.at(left_idx + 1) >
               3 * sigma) {
      ++left_idx;
    }
    while (right_idx + 1 < values.size() &&
           timestamps.at(right_idx + 1) - timestamps.at(target_idx) <
               3 * sigma) {
      ++right_idx;
    }

    double prev_point_gaussian_cdf = 0;
    for (size_t integral_idx = left_idx; integral_idx < right_idx;
         ++integral_idx) {
      const double next_timestamp_midpoint =
          (timestamps.at(integral_idx) + timestamps.at(integral_idx + 1)) / 2.0;
      const double next_gaussian_cdf =
          NormalCdf(next_timestamp_midpoint, timestamps.at(target_idx), sigma);
      result.at(target_idx) += values.at(integral_idx) *
                               (next_gaussian_cdf - prev_point_gaussian_cdf);
      prev_point_gaussian_cdf = next_gaussian_cdf;
    }
    result.at(target_idx) +=
        values.at(right_idx) * (1.0 - prev_point_gaussian_cdf);
  }

  return result;
}
}