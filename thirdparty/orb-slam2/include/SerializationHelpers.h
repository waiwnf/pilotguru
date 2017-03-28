#ifndef INCLUDE_SERIALIZATION_HELPERS_H_
#define INCLUDE_SERIALIZATION_HELPERS_H_

#include <list>
#include <string>
#include <vector>

#include <google/protobuf/repeated_field.h>
#include <opencv2/opencv.hpp>

#include "DBoW2/BowVector.h"
#include "DBoW2/FeatureVector.h"

#include "SerializedBow.pb.h"

namespace ORB_SLAM2 {
std::string SerializeMat(const cv::Mat& mat);
cv::Mat DeserializeMat(const std::string& serialized);
std::list<cv::Mat> DeserializeMatList(const ::google::protobuf::RepeatedPtrField<std::string>& serialized);

std::string SerializeKeyPoints(const std::vector<cv::KeyPoint>& keyPoints);
std::vector<cv::KeyPoint> DeserializeKeyPoints(const std::string& serialized);

std::string SerializePoint2fVec(const std::vector<cv::Point2f>& points);
std::vector<cv::Point2f> DeserializePoint2fVec(const std::string serialized);

std::string SerializePoint3fVec(const std::vector<cv::Point3f>& points);
std::vector<cv::Point3f> DeserializePoint3fVec(const std::string serialized);

template<typename T>
void CopyVectorToRepeatedField(const std::vector<T>& src,
		::google::protobuf::RepeatedField<T>* dest) {
	for (const T x : src) {
		dest->Add(x);
	}
}

template<typename T>
void CopyListToRepeatedField(const std::list<T>& src,
		::google::protobuf::RepeatedField<T>* dest) {
	for (const T x : src) {
		dest->Add(x);
	}
}

DBoW2::FeatureVector FeatureVectorFromProto(
		const ::google::protobuf::RepeatedPtrField<FeatureVectorElement>& elements);

DBoW2::BowVector BowVectorFromProto(
		const ::google::protobuf::RepeatedPtrField<BowVectorElement>& elements);
}

#endif /* INCLUDE_SERIALIZATION_HELPERS_H_ */
