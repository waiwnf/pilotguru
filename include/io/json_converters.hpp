#ifndef PILOTGURU_IO_JSON_CONVERTERS_HPP_
#define PILOTGURU_IO_JSON_CONVERTERS_HPP_

#include <System.h>

#include <json.hpp>
#include <opencv2/core/core.hpp>

namespace pilotguru {
constexpr char kPlane[] = "plane";
constexpr char kTrajectory[] = "trajectory";
constexpr char kTimeUsec[] = "time_usec";
constexpr char kIsLost[] = "is_lost";
constexpr char kFrameId[] = "frame_id";
constexpr char kPose[] = "pose";
constexpr char kPlanarDirection[] = "planar_direction";
constexpr char kTurnAngle[] = "turn_angle";

constexpr char kTranslation[] = "translation";
constexpr char kW[] = "w";
constexpr char kX[] = "x";
constexpr char kY[] = "y";
constexpr char kZ[] = "z";
constexpr char kRotation[] = "rotation";

nlohmann::json PoseToJson(const ORB_SLAM2::Pose &pose);

void SetPlane(nlohmann::json *json_root, const cv::Mat &plane);
cv::Mat ReadPlane(const nlohmann::json& json_root);

void SetTrajectory(
    nlohmann::json *json_root,
    const std::vector<ORB_SLAM2::PoseWithTimestamp> &trajectory,
    const std::vector<cv::Mat> *projected_directions /* optional */,
    const vector<double> *turn_angles /* optional */, int frame_id_offset);

void ReadTrajectory(
    const nlohmann::json &trajectory_json,
    std::vector<ORB_SLAM2::PoseWithTimestamp> *trajectory,
    std::vector<cv::Mat> *projected_directions /* optional */,
    vector<double> *turn_angles /* optional */);
} // namespace pilotguru

#endif // PILOTGURU_IO_JSON_CONVERTERS_HPP_