/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
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

#include <glog/logging.h>

#include "Map.h"

#include "KeyFrame.h"
#include "MapPoint.h"

namespace ORB_SLAM2
{

Map::Map():mnMaxKFid(0)
{
}

void Map::AddKeyFrame(KeyFrame *pKF)
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    mspKeyFrames.insert(pKF);
    if(pKF->mnId>mnMaxKFid)
        mnMaxKFid=pKF->mnId;
}

void Map::AddMapPoint(MapPoint *pMP)
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    mspMapPoints.insert(pMP);
}

void Map::EraseMapPoint(MapPoint *pMP)
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    mspMapPoints.erase(pMP);

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map::EraseKeyFrame(KeyFrame *pKF)
{
	std::unique_lock<std::mutex> lock(mMutexMap);
    mspKeyFrames.erase(pKF);

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map::SetReferenceMapPoints(const std::vector<MapPoint *> &vpMPs)
{
	std::unique_lock<std::mutex> lock(mMutexMap);
    mvpReferenceMapPoints = vpMPs;
}

std::vector<KeyFrame*> Map::GetAllKeyFrames()
{
	std::unique_lock<std::mutex> lock(mMutexMap);
    return vector<KeyFrame*>(mspKeyFrames.begin(),mspKeyFrames.end());
}

std::vector<MapPoint*> Map::GetAllMapPoints()
{
	std::unique_lock<std::mutex> lock(mMutexMap);
    return vector<MapPoint*>(mspMapPoints.begin(),mspMapPoints.end());
}

long unsigned int Map::MapPointsInMap()
{
	std::unique_lock<std::mutex> lock(mMutexMap);
    return mspMapPoints.size();
}

long unsigned int Map::KeyFramesInMap()
{
	std::unique_lock<std::mutex> lock(mMutexMap);
    return mspKeyFrames.size();
}

std::vector<MapPoint*> Map::GetReferenceMapPoints()
{
	std::unique_lock<std::mutex> lock(mMutexMap);
    return mvpReferenceMapPoints;
}

long unsigned int Map::GetMaxKFid()
{
	std::unique_lock<std::mutex> lock(mMutexMap);
    return mnMaxKFid;
}

void Map::clear()
{
    for(std::set<MapPoint*>::iterator sit=mspMapPoints.begin(), send=mspMapPoints.end(); sit!=send; sit++)
        delete *sit;

    for(std::set<KeyFrame*>::iterator sit=mspKeyFrames.begin(), send=mspKeyFrames.end(); sit!=send; sit++)
        delete *sit;

    mspMapPoints.clear();
    mspKeyFrames.clear();
    mnMaxKFid = 0;
    mvpReferenceMapPoints.clear();
    mvpKeyFrameOrigins.clear();
}

void Map::Populate(MapPointersQueue* queue) const {
	std::unique_lock<std::mutex> lock(mMutexMap);
	CHECK_NOTNULL(queue);

	for (KeyFrame* const kf : mspKeyFrames) {
		queue->nodes.insert(CHECK_NOTNULL(kf));
	}
	for (MapPoint* const mp : mspMapPoints) {
		queue->nodes.insert(CHECK_NOTNULL(mp));
	}
	for (MapPoint* const mp : mvpReferenceMapPoints) {
		queue->nodes.insert(CHECK_NOTNULL(mp));
	}
}

SerializedMap Map::Serialize(const MapPointersIndex& pointersIndex) {
	std::unique_lock<std::mutex> lock(mMutexMap);

	SerializedMap result;
	result.set_mnmaxkfid(mnMaxKFid);
	for (MapPoint* const mp : mspMapPoints) {
		result.add_mspmappoints(pointersIndex.points.IndexOrDie(mp));
	}
	for (KeyFrame* const kf : mspKeyFrames) {
		result.add_mspkeyframes(pointersIndex.frames.IndexOrDie(kf));
	}
	for (MapPoint* const mp : mvpReferenceMapPoints) {
		result.add_mvpreferencemappoints(pointersIndex.points.IndexOrDie(mp));
	}

	return result;
}

Map::Map(const SerializedMap& serialized, const IndexedNodes& index) {
	// TODO: make sure the entries in the input serialized proto are unique
	// within each field.
	for (const size_t pointIndex : serialized.mspmappoints()) {
		mspMapPoints.insert(LookupOrDie(pointIndex, index.points));
	}
	for (const size_t frameIndex : serialized.mspkeyframes()) {
		mspKeyFrames.insert(LookupOrDie(frameIndex, index.frames));
	}
	for (const size_t pointIndex : serialized.mvpreferencemappoints()) {
		mvpReferenceMapPoints.push_back(LookupOrDie(pointIndex, index.points));
	}
	mnMaxKFid = serialized.mnmaxkfid();
}

} //namespace ORB_SLAM
