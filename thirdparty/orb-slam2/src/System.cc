/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Ra������l Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include <fcntl.h>
#include <iomanip>
#include <thread>

#include <glog/logging.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <pangolin/pangolin.h>

#include "System.h"
#include "Converter.h"
#include "MapPointersIndex.h"
#include "SerializedMapPoint.pb.h"
#include "SerializedNodes.pb.h"

namespace ORB_SLAM2
{
void LoadSystemProto(const string& systemFile, ORB_SLAM2::SerializedSystem* systemProto) {
	CHECK_NOTNULL(systemProto);
	LOG(INFO) << "Starting to read serialized mapping system.";
	int fd = open(systemFile.c_str(), O_RDONLY);
	CHECK_NE(fd, -1) << "File not found: " << systemFile;
	std::unique_ptr<google::protobuf::io::ZeroCopyInputStream> raw_input(new google::protobuf::io::FileInputStream(fd));
	std::unique_ptr<google::protobuf::io::CodedInputStream> coded_input(new google::protobuf::io::CodedInputStream(raw_input.get()));
	// Maps are typically much larger than the default protocol buffer read
	// size limit of 64MB. Increase the limit to the maximum protobuf size of
	// 2GB to work around this in the short term.
	// TODO: redesign the map storage format to use separate records for individual
	// keyframes and keypoints.
	coded_input->SetTotalBytesLimit(2147483640, 1073741824);
	CHECK(systemProto->ParseFromCodedStream(coded_input.get()));
	close(fd);
	LOG(INFO) << "Mapping system loaded.";
}

System::System(ORBVocabulary* vocabulary, const string &strSettingsFile,
    const eSensor sensor, const bool bUseViewer) :
    mSensor(sensor), mpVocabulary(CHECK_NOTNULL(vocabulary)), is_vocabulary_owned_(
        false), mpMap(new Map()) {
  InitAll(strSettingsFile, bUseViewer);
}

System::System(const string &strVocFile, const string &strSettingsFile,
    const eSensor sensor, const bool bUseViewer) :
    mSensor(sensor), mpVocabulary(new ORBVocabulary(strVocFile)), is_vocabulary_owned_(
        true), mpMap(new Map()) {
  InitAll(strSettingsFile, bUseViewer);
}

void System::InitAll(const std::string& strSettingsFile, const bool bUseViewer) {
  InitPreMap();
  mpKeyFrameDatabase.reset(new KeyFrameDatabase(*mpVocabulary));

  InitDrawers(strSettingsFile);

  //Initialize the Tracking thread
  //(it will live in the main thread of execution, the one that called this constructor)
  mpTracker.reset(
      new Tracking(this, mpVocabulary, mpFrameDrawer.get(),
          mpMapDrawer.get(), mpMap.get(),
          mpKeyFrameDatabase.get(), strSettingsFile, mSensor));

  InitPostMap(strSettingsFile, bUseViewer);

}

System::System(const SerializedSystem& serialized, const string &strVocFile,
		const string &strSettingsFile, const eSensor sensor,
		const bool bUseViewer) :
		mSensor(sensor) {
	InitPreMap();

	// Restore the map points and key frames.
	IndexedNodes indexedNodes = DeserializeNodes(serialized.nodes());
	// Restore the map points and key frames.

	mpKeyFrameDatabase.reset(
			new KeyFrameDatabase(*mpVocabulary, serialized.keyframedatabase(),
					indexedNodes.frames));

	for (auto& indexedFrame : indexedNodes.frames) {
		indexedFrame.second->mpKeyFrameDB = mpKeyFrameDatabase.get();
		indexedFrame.second->mpORBvocabulary = mpVocabulary;
	}

	mpMap.reset(new Map(serialized.map(), indexedNodes));
	for (auto& indexedFrame : indexedNodes.frames) {
		indexedFrame.second->mpMap = mpMap.get();
	}

	InitDrawers(strSettingsFile);

	mpTracker.reset(
			new Tracking(serialized.tracking(), indexedNodes, mpFrameDrawer.get(),
					mpMapDrawer.get(), mpMap.get(), this, mpVocabulary,
					mpKeyFrameDatabase.get(), strSettingsFile));

	InitPostMap(strSettingsFile, bUseViewer);
}

System::~System() {
  if (is_vocabulary_owned_) {
    delete mpVocabulary;
  }
}

void System::InitPreMap() {
    // Output welcome message
    cout << endl <<
    "ORB-SLAM2 Copyright (C) 2014-2016 Raul Mur-Artal, University of Zaragoza." << endl <<
    "This program comes with ABSOLUTELY NO WARRANTY;" << endl  <<
    "This is free software, and you are welcome to redistribute it" << endl <<
    "under certain conditions. See LICENSE.txt." << endl << endl;

    cout << "Input sensor was set to: ";

    if(mSensor==MONOCULAR)
        cout << "Monocular" << endl;
    else if(mSensor==STEREO)
        cout << "Stereo" << endl;
    else if(mSensor==RGBD)
        cout << "RGB-D" << endl;
}

void System::InitDrawers(const string& strSettingsFile) {
	//Create Drawers. These are used by the Viewer
	mpFrameDrawer.reset(new FrameDrawer(mpMap.get()));
	mpMapDrawer.reset(new MapDrawer(mpMap.get(), strSettingsFile));
}

void System::InitPostMap(const string& strSettingsFile, bool bUseViewer) {
	//Check settings file
	cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
	if (!fsSettings.isOpened()) {
		cerr << "Failed to open settings file at: " << strSettingsFile << endl;
		exit(-1);
	}

	//Initialize the Local Mapping thread and launch
	mpLocalMapper.reset(new LocalMapping(mpMap.get(), mSensor == MONOCULAR));
	mptLocalMapping = new thread(&ORB_SLAM2::LocalMapping::Run,
			mpLocalMapper.get());

	//Initialize the Loop Closing thread and launch
	mpLoopCloser.reset(
			new LoopClosing(mpMap.get(), mpKeyFrameDatabase.get(),
					mpVocabulary, mSensor != MONOCULAR));
	mptLoopClosing = new thread(&ORB_SLAM2::LoopClosing::Run,
			mpLoopCloser.get());

	//Initialize the Viewer thread and launch
	mpViewer.reset(
			new Viewer(this, mpFrameDrawer.get(), mpMapDrawer.get(),
					mpTracker.get(), strSettingsFile));
	if (bUseViewer)
		mptViewer = new thread(&Viewer::Run, mpViewer.get());

	mpTracker->SetViewer(mpViewer.get());

	//Set pointers between threads
	mpTracker->SetLocalMapper(mpLocalMapper.get());
	mpTracker->SetLoopClosing(mpLoopCloser.get());

	mpLocalMapper->SetTracker(mpTracker.get());
	mpLocalMapper->SetLoopCloser(mpLoopCloser.get());

	mpLoopCloser->SetTracker(mpTracker.get());
	mpLoopCloser->SetLocalMapper(mpLocalMapper.get());
}

cv::Mat System::TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp)
{
    if(mSensor!=STEREO)
    {
        cerr << "ERROR: you called TrackStereo but input sensor was not set to STEREO." << endl;
        exit(-1);
    }

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
    unique_lock<mutex> lock(mMutexReset);
    if(mbReset)
    {
        mpTracker->Reset();
        mbReset = false;
    }
    }

    return mpTracker->GrabImageStereo(imLeft,imRight,timestamp);
}

cv::Mat System::TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp)
{
    if(mSensor!=RGBD)
    {
        cerr << "ERROR: you called TrackRGBD but input sensor was not set to RGBD." << endl;
        exit(-1);
    }

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
    unique_lock<mutex> lock(mMutexReset);
    if(mbReset)
    {
        mpTracker->Reset();
        mbReset = false;
    }
    }

    return mpTracker->GrabImageRGBD(im,depthmap,timestamp);
}

cv::Mat System::TrackMonocular(const TimestampedImage& frame)
{
	CHECK_EQ(mSensor, MONOCULAR)<< "Called TrackMonocular but input sensor was not set to Monocular.";

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            LOG(INFO) << "Stopped local mapper";

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
    unique_lock<mutex> lock(mMutexReset);
    if(mbReset)
    {
        mpTracker->Reset();
        mbReset = false;
    }
    }

    return mpTracker->GrabImageMonocular(frame);
}

void System::ActivateLocalizationMode()
{
    unique_lock<mutex> lock(mMutexMode);
    mbActivateLocalizationMode = true;
}

void System::DeactivateLocalizationMode()
{
    unique_lock<mutex> lock(mMutexMode);
    mbDeactivateLocalizationMode = true;
}

void System::Reset()
{
    unique_lock<mutex> lock(mMutexReset);
    mbReset = true;
}

void System::Shutdown()
{
    mpLocalMapper->RequestFinish();
    mpLoopCloser->RequestFinish();
    mpViewer->RequestFinish();

    // Wait until all thread have effectively stopped
    // mpLoopCloser->isRunningGBA() was also allowed previously.
    while(!mpLocalMapper->isFinished() || !mpLoopCloser->isFinished()  ||
          !mpViewer->isFinished()      || mpLoopCloser->isRunningGBA())
    {
        usleep(5000);
    }

    mptLocalMapping->join();
    delete mptLocalMapping;

    mptLoopClosing->join();
    delete mptLoopClosing;

    if (mptViewer != nullptr) {
    	mptViewer->join();
    	delete mptViewer;
    }


    // pangolin::BindToContext("ORB-SLAM2: Map Viewer");
}

std::vector<PoseWithTimestamp> System::GetTrajectory() {
  std::vector<PoseWithTimestamp> result;

  vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
  sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

  // Transform all keyframes so that the first keyframe is at the origin.
  // After a loop closure the first keyframe might not be at the origin.
  cv::Mat Two = vpKFs[0]->GetPoseInverse();

  // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
  // We need to get first the keyframe pose and then concatenate the relative transformation.
  // Frames not localized (tracking failure) are not saved.
  for (const RelativeFramePoseData& relative_pose : mpTracker->mlTrajectory) {
    result.push_back(
        { Pose(), static_cast<int64>(relative_pose.mFrameTime
            * 1e6), relative_pose.mbLost, relative_pose.mFrameId });
    if (relative_pose.mbLost) {
      continue;
    }

    KeyFrame* pKF = relative_pose.mpReference;

    cv::Mat Trw = cv::Mat::eye(4, 4, CV_32F);

    // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
    while (pKF->isBad()) {
      Trw = Trw * pKF->mTcp;
      pKF = pKF->GetParent();
    }

    Trw = Trw * pKF->GetPose() * Two;

    const cv::Mat Tcw = relative_pose.mRelativeFramePose * Trw;
    const cv::Mat rotation_mat = Tcw.rowRange(0, 3).colRange(0, 3).t();
    result.back().pose.rotation = Converter::toEigenQuaternion(rotation_mat);
    const cv::Mat translation_mat = -rotation_mat * Tcw.rowRange(0, 3).col(3);
    result.back().pose.translation = cv::Vec3d(translation_mat.at<float>(0, 0),
        translation_mat.at<float>(0, 1), translation_mat.at<float>(0, 2));
  }
  return result;
}

void System::SaveTrajectoryTUM(const string &filename) {
  cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
  const std::vector<PoseWithTimestamp> trajectory = GetTrajectory();

  ofstream f;
  f.open(filename.c_str());
  f << fixed;

  for (const PoseWithTimestamp& pose_data : trajectory) {
    if (pose_data.is_lost) {
      continue;
    }
    const Pose& pose = pose_data.pose;
    f << setprecision(6) << pose_data.time_usec << " " << setprecision(9)
        << pose.translation(0) << " " << pose.translation(1)
        << " " << pose.translation(2) << " " << pose.rotation.x()
        << " " << pose.rotation.y() << " " << pose.rotation.z() << " "
        << pose.rotation.w() << std::endl;
  }
  f.close();
  cout << endl << "trajectory saved!" << endl;
}


void System::SaveKeyFrameTrajectoryTUM(const string &filename)
{
    cout << endl << "Saving keyframe trajectory to " << filename << " ..." << endl;

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    //cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];

       // pKF->SetPose(pKF->GetPose()*Two);

        if(pKF->isBad())
            continue;

        cv::Mat R = pKF->GetRotation().t();
        const vector<double> q = Converter::toQuaternion(R);
        cv::Mat t = pKF->GetCameraCenter();
        f << setprecision(6) << pKF->mTimeStamp << setprecision(7) << " " << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2)
          << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
    }

    f.close();
    cout << endl << "trajectory saved!" << endl;
}

void System::SaveTrajectoryKITTI(const string &filename) {
  cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
  const std::vector<PoseWithTimestamp> trajectory = GetTrajectory();

  ofstream f;
  f.open(filename.c_str());
  f << fixed;

  for (const PoseWithTimestamp& pose_data : trajectory) {
    if (pose_data.is_lost) {
      continue;
    }
    const Pose& pose = pose_data.pose;
    const Eigen::Matrix3d rotation_matrix = pose.rotation.toRotationMatrix();
    f << setprecision(9) << rotation_matrix(0, 0) << " "
        << rotation_matrix(0, 1) << " " << rotation_matrix(0, 2) << " "
        << pose.translation(0) << " " << rotation_matrix(1, 0) << " "
        << rotation_matrix(1, 1) << " " << rotation_matrix(1, 2) << " "
        << pose.translation(1) << " " << rotation_matrix(2, 0) << " "
        << rotation_matrix(2, 1) << " " << rotation_matrix(2, 2) << " "
        << pose.translation(2) << std::endl;
  }
  f.close();
  cout << endl << "trajectory saved!" << endl;
}

SerializedSystem System::Serialize() {
	// TODO make sure nothing is running.

	SerializedSystem result;

	MapPointersQueue pointersQueue;
	mpMap->Populate(&pointersQueue);
	mpTracker->Populate(&pointersQueue);
	const MapPointersIndex pointersIndex = IndexReachableMapPointers(&pointersQueue);

	for (const auto& p : pointersIndex.points.GetToIndex()) {
		IndexedSerializedMapPoint* point = result.mutable_nodes()->add_points();
		point->set_index(p.second);
		*(point->mutable_point()) = p.first->ToIndexedPoint(pointersIndex);
	}
	for (const auto& f : pointersIndex.frames.GetToIndex()) {
		IndexedSerializedKeyFrame* frame = result.mutable_nodes()->add_frames();
		frame->set_index(f.second);
		*(frame->mutable_frame()) = f.first->ToIndexedFrame(pointersIndex);
	}
	*(result.mutable_map()) = mpMap->Serialize(pointersIndex);
	*(result.mutable_keyframedatabase()) = mpKeyFrameDatabase->Serialize(pointersIndex);
	*(result.mutable_tracking()) = mpTracker->Serialize(pointersIndex);

	return result;
}

} //namespace ORB_SLAM
