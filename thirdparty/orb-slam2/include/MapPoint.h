/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Ra��l Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
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

#ifndef MAPPOINT_H
#define MAPPOINT_H

#include <map>
#include <mutex>
#include <set>

#include <glog/logging.h>
#include <opencv2/core/core.hpp>

#include "MapPointersIndex.h"

namespace ORB_SLAM2
{

class KeyFrame;
class Map;
class MapPoint;
class Frame;
class SerializedMapPoint;


class MapPoint : public MapPointersNodeExpander
{
public:
	friend class Map;

    MapPoint(const cv::Mat &Pos, KeyFrame* pRefKF, Map* pMap);
    MapPoint(const cv::Mat &Pos,  Map* pMap, Frame* pFrame, const int &idxF);
    MapPoint(const SerializedMapPoint& serialized);

    void SetWorldPos(const cv::Mat &Pos);
    cv::Mat GetWorldPos();

    cv::Mat GetNormal();
    KeyFrame* GetReferenceKeyFrame();

    std::map<KeyFrame*,size_t> GetObservations();
    int Observations();

    void AddObservation(KeyFrame* pKF,size_t idx);
    void EraseObservation(KeyFrame* pKF);

    int GetIndexInKeyFrame(KeyFrame* pKF);
    bool IsInKeyFrame(KeyFrame* pKF);

    void SetBadFlag();
    bool isBad();

    void Replace(MapPoint* pMP);    
    MapPoint* GetReplaced();

    void IncreaseVisible(int n=1);
    void IncreaseFound(int n=1);
    float GetFoundRatio();
    inline int GetFound(){
        return mnFound;
    }

    void ComputeDistinctiveDescriptors();

    cv::Mat GetDescriptor();

    void UpdateNormalAndDepth();

    float GetMinDistanceInvariance();
    float GetMaxDistanceInvariance();
    int PredictScale(const float &currentDist, KeyFrame*pKF);
    int PredictScale(const float &currentDist, Frame* pF);

	void ExpandNode(MapPointersIndex* alreadyExpanded,
			MapPointersQueue* queue) const override;

	SerializedMapPoint ToIndexedPoint(const MapPointersIndex& index) const;
	void RestorePointers(const SerializedMapPoint& serializedThis, const IndexedNodes& pointersIndex);

private:
	// Allocates a new unique MapPoint ID in a thread-safe way.
	static long unsigned int MakeNextId();

public:
    const long unsigned int mnId;
    const long int mnFirstKFid;
    const long int mnFirstFrame;
    int nObs;

    // Variables used by the tracking
    float mTrackProjX;
    float mTrackProjY;
    float mTrackProjXR;
    bool mbTrackInView = false;  // TODO: make sure this is initialized properly.
    int mnTrackScaleLevel = 0;  // TODO: make sure this is initialized properly.
    float mTrackViewCos;
    long unsigned int mnTrackReferenceForFrame;
    long unsigned int mnLastFrameSeen;

    // Variables used by local mapping
    long unsigned int mnBALocalForKF;
    long unsigned int mnFuseCandidateForKF;

    // Variables used by loop closing
    long unsigned int mnLoopPointForKF;
    long unsigned int mnCorrectedByKF;
    long unsigned int mnCorrectedReference;    
    cv::Mat mPosGBA;
    long unsigned int mnBAGlobalForKF;


    static std::mutex mGlobalMutex;

protected:    

     // Position in absolute coordinates
     cv::Mat mWorldPos;
public:
     // Keyframes observing the point and associated index in keyframe
     std::map<KeyFrame*,size_t> mObservations;
protected:
     // Mean viewing direction
     cv::Mat mNormalVector;

     // Best descriptor to fast matching
     cv::Mat mDescriptor;
public:
     // Reference KeyFrame
     KeyFrame* mpRefKF;
protected:
     // Tracking counters
     int mnVisible;
     int mnFound;

     // Bad flag (we do not currently erase MapPoint from memory)
     bool mbBad;
public:
     MapPoint* mpReplaced;
protected:
     // Scale invariance distances
     float mfMinDistance;
     float mfMaxDistance;

     std::mutex mMutexPos;
     std::mutex mMutexFeatures;
};
} //namespace ORB_SLAM

#endif // MAPPOINT_H
