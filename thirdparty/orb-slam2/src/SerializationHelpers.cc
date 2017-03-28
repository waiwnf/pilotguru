#include "SerializationHelpers.h"

namespace ORB_SLAM2 {
cv::Mat DeserializeMat(const std::string& serialized) {
	cv::FileStorage fs(serialized,
			cv::FileStorage::READ | cv::FileStorage::MEMORY);
	cv::Mat result;
	fs["data"] >> result;
	return result;
}

std::list<cv::Mat> DeserializeMatList(
		const ::google::protobuf::RepeatedPtrField<std::string>& serialized) {
	std::list<cv::Mat> result;
	for (const std::string& s : serialized) {
		result.push_back(DeserializeMat(s));
	}
	return result;
}

std::string SerializeMat(const cv::Mat& mat) {
	cv::FileStorage fs("data.yml",
			cv::FileStorage::WRITE | cv::FileStorage::MEMORY);
	cv::write(fs, "data", mat);
	return fs.releaseAndGetString();
}

std::vector<cv::KeyPoint> DeserializeKeyPoints(const std::string& serialized) {
	cv::FileStorage fs(serialized,
			cv::FileStorage::READ | cv::FileStorage::MEMORY);
	std::vector<cv::KeyPoint> result;
	cv::read(fs["data"], result);
	return result;
}

std::string SerializeKeyPoints(const std::vector<cv::KeyPoint>& keyPoints) {
	cv::FileStorage fs("data.yml",
			cv::FileStorage::WRITE | cv::FileStorage::MEMORY);
	cv::write(fs, "data", keyPoints);
	return fs.releaseAndGetString();
}

std::string SerializePoint2fVec(const std::vector<cv::Point2f>& points) {
	cv::FileStorage fs("data.yml",
			cv::FileStorage::WRITE | cv::FileStorage::MEMORY);
	cv::write(fs, "data", points);
	return fs.releaseAndGetString();
}

std::vector<cv::Point2f> DeserializePoint2fVec(const std::string serialized) {
	cv::FileStorage fs(serialized,
			cv::FileStorage::READ | cv::FileStorage::MEMORY);
	std::vector<cv::Point2f> result;
	cv::read(fs["data"], result);
	return result;
}

std::string SerializePoint3fVec(const std::vector<cv::Point3f>& points) {
	cv::FileStorage fs("data.yml",
			cv::FileStorage::WRITE | cv::FileStorage::MEMORY);
	cv::write(fs, "data", points);
	return fs.releaseAndGetString();
}

std::vector<cv::Point3f> DeserializePoint3fVec(const std::string serialized) {
	cv::FileStorage fs(serialized,
			cv::FileStorage::READ | cv::FileStorage::MEMORY);
	std::vector<cv::Point3f> result;
	cv::read(fs["data"], result);
	return result;
}

DBoW2::FeatureVector FeatureVectorFromProto(
		const ::google::protobuf::RepeatedPtrField<FeatureVectorElement>& elements) {
	DBoW2::FeatureVector result;
	for (const FeatureVectorElement& element : elements) {
		for (const unsigned int feature : element.features()) {
			result.addFeature(element.nodeid(), feature);
		}
	}
	return result;
}

DBoW2::BowVector BowVectorFromProto(
		const ::google::protobuf::RepeatedPtrField<BowVectorElement>& elements) {
	DBoW2::BowVector result;
	for (const BowVectorElement& element : elements) {
		result.addWeight(element.wordid(), element.wordvalue());
	}
	return result;
}

}
