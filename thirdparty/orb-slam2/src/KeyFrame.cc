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

#include "KeyFrame.h"

#include "Converter.h"
#include "Frame.h"
#include "KeyFrameDatabase.h"
#include "Map.h"
#include "MapPoint.h"
#include "SerializationHelpers.h"
#include "SerializedKeyFrame.pb.h"

namespace ORB_SLAM2
{

namespace {
std::vector<std::vector<std::vector<size_t> > > GridFromProto(
		const ::google::protobuf::RepeatedPtrField<Grid2D>& in) {
	std::vector<std::vector<std::vector<size_t>>>result;
	for (const Grid2D& g2dProto : in) {
		std::vector<std::vector<size_t>> g2d;
		for (const Grid1D& g1dProto : g2dProto.values()) {
			g2d.emplace_back(g1dProto.values().begin(), g1dProto.values().end());
		}
		result.push_back(g2d);
	}
	return result;
}

void FillProtoFromGrid(
		const std::vector<std::vector<std::vector<size_t> > >& grid,
		::google::protobuf::RepeatedPtrField<Grid2D>* dest) {
	for (const std::vector<std::vector<size_t> >& g2d : grid) {
		Grid2D* g2dProto = dest->Add();
		for (const std::vector<size_t>& g1d : g2d) {
			Grid1D* g1dProto = g2dProto->add_values();
			for (const size_t v : g1d) {
				g1dProto->add_values(v);
			}
		}
	}
}
}  // namespace

long unsigned int KeyFrame::nNextId=0;

KeyFrame::KeyFrame(const SerializedKeyFrame& serialized) :
    mnId(serialized.mnid()),
    mnFrameId(serialized.mnframeid()),
    mTimeStamp(serialized.mtimestamp()),
    mnGridCols(serialized.mngridcols()),
    mnGridRows(serialized.mngridrows()),
    mfGridElementWidthInv(serialized.mfgridelementwidthinv()),
    mfGridElementHeightInv(serialized.mfgridelementheightinv()),
    mnTrackReferenceForFrame(serialized.mntrackreferenceforframe()),
    mnFuseTargetForKF(serialized.mnfusetargetforkf()),
    mnBALocalForKF(serialized.mnbalocalforkf()),
    mnBAFixedForKF(serialized.mnbafixedforkf()),
    mnLoopQuery(serialized.mnloopquery()),
    mnLoopWords(serialized.mnloopwords()),
    mLoopScore(serialized.mloopscore()),
    mnRelocQuery(serialized.mnrelocquery()),
    mnRelocWords(serialized.mnrelocwords()),
    mRelocScore(serialized.mrelocscore()),
    mTcwGBA(DeserializeMat(serialized.mtcwgba())),
    mTcwBefGBA(DeserializeMat(serialized.mtcwbefgba())),
    mnBAGlobalForKF(serialized.mnbaglobalforkf()),
    fx(serialized.fx()),
	fy(serialized.fy()),
	cx(serialized.cx()),
	cy(serialized.cy()),
	invfx(serialized.invfx()),
	invfy(serialized.invfy()),
	mbf(serialized.mbf()),
	mb(serialized.mb()),
	mThDepth(serialized.mthdepth()),
    N(serialized.n()),
    mvKeys(DeserializeKeyPoints(serialized.mvkeys())),
    mvKeysUn(DeserializeKeyPoints(serialized.mvkeysun())),
    mvuRight(serialized.mvuright().begin(), serialized.mvuright().end()),
    mvDepth(serialized.mvdepth().begin(), serialized.mvdepth().end()),
    mDescriptors(DeserializeMat(serialized.mdescriptors())),
    mBowVec(BowVectorFromProto(serialized.mbowvec())),
    mFeatVec(FeatureVectorFromProto(serialized.mfeatvec())),
    mTcp(DeserializeMat(serialized.mtcp())),
    mnScaleLevels(serialized.mnscalelevels()),
    mfScaleFactor(serialized.mfscalefactor()),
    mfLogScaleFactor(serialized.mflogscalefactor()),
    mvScaleFactors(serialized.mvscalefactors().begin(), serialized.mvscalefactors().end()),
    mvLevelSigma2(serialized.mvlevelsigma2().begin(), serialized.mvlevelsigma2().end()),
    mvInvLevelSigma2(serialized.mvinvlevelsigma2().begin(), serialized.mvinvlevelsigma2().end()),
    mnMinX(serialized.mnminx()),
    mnMinY(serialized.mnminy()),
    mnMaxX(serialized.mnmaxx()),
    mnMaxY(serialized.mnmaxy()),
    mK(DeserializeMat(serialized.mk())),
    Tcw(DeserializeMat(serialized.tcw())),
    Twc(DeserializeMat(serialized.twc())),
    Ow(DeserializeMat(serialized.ow())),
    Cw(DeserializeMat(serialized.cw())),
    mpKeyFrameDB(nullptr),
    mpORBvocabulary(nullptr),
    mGrid(GridFromProto(serialized.mgrid())),
    mvOrderedWeights(serialized.mvorderedweights().begin(), serialized.mvorderedweights().end()),
    mbFirstConnection(serialized.mbfirstconnection()),
    mpParent(nullptr),
    mbNotErase(serialized.mbnoterase()),
    mbToBeErased(serialized.mbtobeerased()),
    mbBad(serialized.mbbad()),
    mHalfBaseline(serialized.mhalfbaseline()),
    mpMap(nullptr)
{
	KeyFrame::nNextId = serialized.nnextid();
}

KeyFrame::KeyFrame(Frame &F, Map *pMap, KeyFrameDatabase *pKFDB):
    mnFrameId(F.mnId),  mTimeStamp(F.mTimeStamp), mnGridCols(FRAME_GRID_COLS), mnGridRows(FRAME_GRID_ROWS),
    mfGridElementWidthInv(F.mfGridElementWidthInv), mfGridElementHeightInv(F.mfGridElementHeightInv),
    mnTrackReferenceForFrame(0), mnFuseTargetForKF(0), mnBALocalForKF(0), mnBAFixedForKF(0),
    mnLoopQuery(0), mnLoopWords(0), mnRelocQuery(0), mnRelocWords(0), mnBAGlobalForKF(0),
    fx(F.fx), fy(F.fy), cx(F.cx), cy(F.cy), invfx(F.invfx), invfy(F.invfy),
    mbf(F.mbf), mb(F.mb), mThDepth(F.mThDepth), N(F.N), mvKeys(F.mvKeys), mvKeysUn(F.mvKeysUndistorted),
    mvuRight(F.mvuRight), mvDepth(F.mvDepth), mDescriptors(F.mDescriptors.clone()),
    mBowVec(F.mBowVec), mFeatVec(F.mFeatVec), mnScaleLevels(F.mnScaleLevels), mfScaleFactor(F.mfScaleFactor),
    mfLogScaleFactor(F.mfLogScaleFactor), mvScaleFactors(F.mvScaleFactors), mvLevelSigma2(F.mvLevelSigma2),
    mvInvLevelSigma2(F.mvInvLevelSigma2), mnMinX(F.mnMinX), mnMinY(F.mnMinY), mnMaxX(F.mnMaxX),
    mnMaxY(F.mnMaxY), mK(F.mK), mvpMapPoints(F.mvpMapPoints), mpKeyFrameDB(pKFDB),
    mpORBvocabulary(F.mpORBvocabulary), mbFirstConnection(true), mpParent(NULL), mbNotErase(false),
    mbToBeErased(false), mbBad(false), mHalfBaseline(F.mb/2), mpMap(pMap)
{
    mnId=nNextId++;

    mGrid.resize(mnGridCols);
    for(int i=0; i<mnGridCols;i++)
    {
        mGrid[i].resize(mnGridRows);
        for(int j=0; j<mnGridRows; j++)
            mGrid[i][j] = F.mGrid[i][j];
    }

    SetPose(F.mTcw);    
}

void KeyFrame::ComputeBoW()
{
    if(mBowVec.empty() || mFeatVec.empty())
    {
        vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(mDescriptors);
        // Feature vector associate features with nodes in the 4th level (from leaves up)
        // We assume the vocabulary tree has 6 levels, change the 4 otherwise
        mpORBvocabulary->transform(vCurrentDesc,mBowVec,mFeatVec,4);
    }
}

void KeyFrame::SetPose(const cv::Mat &Tcw_)
{
    std::unique_lock<mutex> lock(mMutexPose);
    Tcw_.copyTo(Tcw);
    cv::Mat Rcw = Tcw.rowRange(0,3).colRange(0,3);
    cv::Mat tcw = Tcw.rowRange(0,3).col(3);
    cv::Mat Rwc = Rcw.t();
    Ow = -Rwc*tcw;

    Twc = cv::Mat::eye(4,4,Tcw.type());
    Rwc.copyTo(Twc.rowRange(0,3).colRange(0,3));
    Ow.copyTo(Twc.rowRange(0,3).col(3));
    cv::Mat center = (cv::Mat_<float>(4,1) << mHalfBaseline, 0 , 0, 1);
    Cw = Twc*center;
}

cv::Mat KeyFrame::GetPose()
{
    std::unique_lock<mutex> lock(mMutexPose);
    return Tcw.clone();
}

cv::Mat KeyFrame::GetPoseInverse()
{
    std::unique_lock<mutex> lock(mMutexPose);
    return Twc.clone();
}

cv::Mat KeyFrame::GetCameraCenter()
{
    std::unique_lock<mutex> lock(mMutexPose);
    return Ow.clone();
}

cv::Mat KeyFrame::GetStereoCenter()
{
    std::unique_lock<mutex> lock(mMutexPose);
    return Cw.clone();
}


cv::Mat KeyFrame::GetRotation()
{
    std::unique_lock<mutex> lock(mMutexPose);
    return Tcw.rowRange(0,3).colRange(0,3).clone();
}

cv::Mat KeyFrame::GetTranslation()
{
    std::unique_lock<mutex> lock(mMutexPose);
    return Tcw.rowRange(0,3).col(3).clone();
}

void KeyFrame::AddConnection(KeyFrame *pKF, const int &weight)
{
    {
        std::unique_lock<mutex> lock(mMutexConnections);
        if(!mConnectedKeyFrameWeights.count(pKF))
            mConnectedKeyFrameWeights[pKF]=weight;
        else if(mConnectedKeyFrameWeights[pKF]!=weight)
            mConnectedKeyFrameWeights[pKF]=weight;
        else
            return;
    }

    UpdateBestCovisibles();
}

void KeyFrame::UpdateBestCovisibles()
{
    std::unique_lock<mutex> lock(mMutexConnections);
    vector<pair<int,KeyFrame*> > vPairs;
    vPairs.reserve(mConnectedKeyFrameWeights.size());
    for(map<KeyFrame*,int>::iterator mit=mConnectedKeyFrameWeights.begin(), mend=mConnectedKeyFrameWeights.end(); mit!=mend; mit++)
       vPairs.push_back(make_pair(mit->second,mit->first));

    sort(vPairs.begin(),vPairs.end());
    list<KeyFrame*> lKFs;
    list<int> lWs;
    for(size_t i=0, iend=vPairs.size(); i<iend;i++)
    {
        lKFs.push_front(vPairs[i].second);
        lWs.push_front(vPairs[i].first);
    }

    mvpOrderedConnectedKeyFrames = vector<KeyFrame*>(lKFs.begin(),lKFs.end());
    mvOrderedWeights = vector<int>(lWs.begin(), lWs.end());    
}

set<KeyFrame*> KeyFrame::GetConnectedKeyFrames()
{
    std::unique_lock<mutex> lock(mMutexConnections);
    set<KeyFrame*> s;
    for(map<KeyFrame*,int>::iterator mit=mConnectedKeyFrameWeights.begin();mit!=mConnectedKeyFrameWeights.end();mit++)
        s.insert(mit->first);
    return s;
}

vector<KeyFrame*> KeyFrame::GetVectorCovisibleKeyFrames()
{
    std::unique_lock<mutex> lock(mMutexConnections);
    return mvpOrderedConnectedKeyFrames;
}

vector<KeyFrame*> KeyFrame::GetBestCovisibilityKeyFrames(const int &N)
{
    std::unique_lock<mutex> lock(mMutexConnections);
    if((int)mvpOrderedConnectedKeyFrames.size()<N)
        return mvpOrderedConnectedKeyFrames;
    else
        return vector<KeyFrame*>(mvpOrderedConnectedKeyFrames.begin(),mvpOrderedConnectedKeyFrames.begin()+N);

}

vector<KeyFrame*> KeyFrame::GetCovisiblesByWeight(const int &w)
{
    std::unique_lock<mutex> lock(mMutexConnections);

    if(mvpOrderedConnectedKeyFrames.empty())
        return vector<KeyFrame*>();

    vector<int>::iterator it = upper_bound(mvOrderedWeights.begin(),mvOrderedWeights.end(),w,KeyFrame::weightComp);
    if(it==mvOrderedWeights.end())
        return vector<KeyFrame*>();
    else
    {
        int n = it-mvOrderedWeights.begin();
        return vector<KeyFrame*>(mvpOrderedConnectedKeyFrames.begin(), mvpOrderedConnectedKeyFrames.begin()+n);
    }
}

int KeyFrame::GetWeight(KeyFrame *pKF)
{
    std::unique_lock<mutex> lock(mMutexConnections);
    if(mConnectedKeyFrameWeights.count(pKF))
        return mConnectedKeyFrameWeights[pKF];
    else
        return 0;
}

void KeyFrame::AddMapPoint(MapPoint *pMP, const size_t &idx)
{
    std::unique_lock<mutex> lock(mMutexFeatures);
    mvpMapPoints[idx]=pMP;
}

void KeyFrame::EraseMapPointMatch(const size_t &idx)
{
    std::unique_lock<mutex> lock(mMutexFeatures);
    mvpMapPoints[idx]=static_cast<MapPoint*>(NULL);
}

void KeyFrame::EraseMapPointMatch(MapPoint* pMP)
{
    int idx = pMP->GetIndexInKeyFrame(this);
    if(idx>=0)
        mvpMapPoints[idx]=static_cast<MapPoint*>(NULL);
}


void KeyFrame::ReplaceMapPointMatch(const size_t &idx, MapPoint* pMP)
{
    mvpMapPoints[idx]=pMP;
}

set<MapPoint*> KeyFrame::GetMapPoints()
{
    std::unique_lock<mutex> lock(mMutexFeatures);
    set<MapPoint*> s;
    for(size_t i=0, iend=mvpMapPoints.size(); i<iend; i++)
    {
        if(!mvpMapPoints[i])
            continue;
        MapPoint* pMP = mvpMapPoints[i];
        if(!pMP->isBad())
            s.insert(pMP);
    }
    return s;
}

int KeyFrame::TrackedMapPoints(const int &minObs)
{
    std::unique_lock<mutex> lock(mMutexFeatures);

    int nPoints=0;
    const bool bCheckObs = minObs>0;
    for(int i=0; i<N; i++)
    {
        MapPoint* pMP = mvpMapPoints[i];
        if(pMP)
        {
            if(!pMP->isBad())
            {
                if(bCheckObs)
                {
                    if(mvpMapPoints[i]->Observations()>=minObs)
                        nPoints++;
                }
                else
                    nPoints++;
            }
        }
    }

    return nPoints;
}

vector<MapPoint*> KeyFrame::GetMapPointMatches()
{
    std::unique_lock<mutex> lock(mMutexFeatures);
    return mvpMapPoints;
}

MapPoint* KeyFrame::GetMapPoint(const size_t &idx)
{
    std::unique_lock<mutex> lock(mMutexFeatures);
    return mvpMapPoints[idx];
}

void KeyFrame::UpdateConnections()
{
    map<KeyFrame*,int> KFcounter;

    vector<MapPoint*> vpMP;

    {
        std::unique_lock<mutex> lockMPs(mMutexFeatures);
        vpMP = mvpMapPoints;
    }

    //For all map points in keyframe check in which other keyframes are they seen
    //Increase counter for those keyframes
    for(vector<MapPoint*>::iterator vit=vpMP.begin(), vend=vpMP.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;

        if(!pMP)
            continue;

        if(pMP->isBad())
            continue;

        map<KeyFrame*,size_t> observations = pMP->GetObservations();

        for(map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            if(mit->first->mnId==mnId)
                continue;
            KFcounter[mit->first]++;
        }
    }

    // This should not happen
    if(KFcounter.empty())
        return;

    //If the counter is greater than threshold add connection
    //In case no keyframe counter is over threshold add the one with maximum counter
    int nmax=0;
    KeyFrame* pKFmax=NULL;
    int th = 15;

    vector<pair<int,KeyFrame*> > vPairs;
    vPairs.reserve(KFcounter.size());
    for(map<KeyFrame*,int>::iterator mit=KFcounter.begin(), mend=KFcounter.end(); mit!=mend; mit++)
    {
        if(mit->second>nmax)
        {
            nmax=mit->second;
            pKFmax=mit->first;
        }
        if(mit->second>=th)
        {
            vPairs.push_back(make_pair(mit->second,mit->first));
            (mit->first)->AddConnection(this,mit->second);
        }
    }

    if(vPairs.empty())
    {
        vPairs.push_back(make_pair(nmax,pKFmax));
        pKFmax->AddConnection(this,nmax);
    }

    sort(vPairs.begin(),vPairs.end());
    list<KeyFrame*> lKFs;
    list<int> lWs;
    for(size_t i=0; i<vPairs.size();i++)
    {
        lKFs.push_front(vPairs[i].second);
        lWs.push_front(vPairs[i].first);
    }

    {
        std::unique_lock<mutex> lockCon(mMutexConnections);

        // mspConnectedKeyFrames = spConnectedKeyFrames;
        mConnectedKeyFrameWeights = KFcounter;
        mvpOrderedConnectedKeyFrames = vector<KeyFrame*>(lKFs.begin(),lKFs.end());
        mvOrderedWeights = vector<int>(lWs.begin(), lWs.end());

        if(mbFirstConnection && mnId!=0)
        {
            mpParent = mvpOrderedConnectedKeyFrames.front();
            mpParent->AddChild(this);
            mbFirstConnection = false;
        }

    }
}

void KeyFrame::AddChild(KeyFrame *pKF)
{
    std::unique_lock<mutex> lockCon(mMutexConnections);
    mspChildrens.insert(pKF);
}

void KeyFrame::EraseChild(KeyFrame *pKF)
{
    std::unique_lock<mutex> lockCon(mMutexConnections);
    mspChildrens.erase(pKF);
}

void KeyFrame::ChangeParent(KeyFrame *pKF)
{
    std::unique_lock<mutex> lockCon(mMutexConnections);
    mpParent = pKF;
    pKF->AddChild(this);
}

set<KeyFrame*> KeyFrame::GetChilds()
{
    std::unique_lock<mutex> lockCon(mMutexConnections);
    return mspChildrens;
}

KeyFrame* KeyFrame::GetParent()
{
    std::unique_lock<mutex> lockCon(mMutexConnections);
    return mpParent;
}

bool KeyFrame::hasChild(KeyFrame *pKF)
{
    std::unique_lock<mutex> lockCon(mMutexConnections);
    return mspChildrens.count(pKF);
}

void KeyFrame::AddLoopEdge(KeyFrame *pKF)
{
    std::unique_lock<mutex> lockCon(mMutexConnections);
    mbNotErase = true;
    mspLoopEdges.insert(pKF);
}

set<KeyFrame*> KeyFrame::GetLoopEdges()
{
    std::unique_lock<mutex> lockCon(mMutexConnections);
    return mspLoopEdges;
}

void KeyFrame::SetNotErase()
{
    std::unique_lock<mutex> lock(mMutexConnections);
    mbNotErase = true;
}

void KeyFrame::SetErase()
{
    {
        std::unique_lock<mutex> lock(mMutexConnections);
        if(mspLoopEdges.empty())
        {
            mbNotErase = false;
        }
    }

    if(mbToBeErased)
    {
        SetBadFlag();
    }
}

void KeyFrame::SetBadFlag()
{   
    {
        std::unique_lock<mutex> lock(mMutexConnections);
        if(mnId==0)
            return;
        else if(mbNotErase)
        {
            mbToBeErased = true;
            return;
        }
    }

    for(map<KeyFrame*,int>::iterator mit = mConnectedKeyFrameWeights.begin(), mend=mConnectedKeyFrameWeights.end(); mit!=mend; mit++)
        mit->first->EraseConnection(this);

    for(size_t i=0; i<mvpMapPoints.size(); i++)
        if(mvpMapPoints[i])
            mvpMapPoints[i]->EraseObservation(this);
    {
        std::unique_lock<mutex> lock(mMutexConnections);
        std::unique_lock<mutex> lock1(mMutexFeatures);

        mConnectedKeyFrameWeights.clear();
        mvpOrderedConnectedKeyFrames.clear();

        // Update Spanning Tree
        set<KeyFrame*> sParentCandidates;
        sParentCandidates.insert(mpParent);

        // Assign at each iteration one children with a parent (the pair with highest covisibility weight)
        // Include that children as new parent candidate for the rest
        while(!mspChildrens.empty())
        {
            bool bContinue = false;

            int max = -1;
            KeyFrame* pC;
            KeyFrame* pP;

            for(set<KeyFrame*>::iterator sit=mspChildrens.begin(), send=mspChildrens.end(); sit!=send; sit++)
            {
                KeyFrame* pKF = *sit;
                if(pKF->isBad())
                    continue;

                // Check if a parent candidate is connected to the keyframe
                vector<KeyFrame*> vpConnected = pKF->GetVectorCovisibleKeyFrames();
                for(size_t i=0, iend=vpConnected.size(); i<iend; i++)
                {
                    for(set<KeyFrame*>::iterator spcit=sParentCandidates.begin(), spcend=sParentCandidates.end(); spcit!=spcend; spcit++)
                    {
                        if(vpConnected[i]->mnId == (*spcit)->mnId)
                        {
                            int w = pKF->GetWeight(vpConnected[i]);
                            if(w>max)
                            {
                                pC = pKF;
                                pP = vpConnected[i];
                                max = w;
                                bContinue = true;
                            }
                        }
                    }
                }
            }

            if(bContinue)
            {
                pC->ChangeParent(pP);
                sParentCandidates.insert(pC);
                mspChildrens.erase(pC);
            }
            else
                break;
        }

        // If a children has no covisibility links with any parent candidate, assign to the original parent of this KF
        if(!mspChildrens.empty())
            for(set<KeyFrame*>::iterator sit=mspChildrens.begin(); sit!=mspChildrens.end(); sit++)
            {
                (*sit)->ChangeParent(mpParent);
            }

        mpParent->EraseChild(this);
        mTcp = Tcw*mpParent->GetPoseInverse();
        mbBad = true;
    }


    mpMap->EraseKeyFrame(this);
    mpKeyFrameDB->erase(this);
}

bool KeyFrame::isBad()
{
    std::unique_lock<mutex> lock(mMutexConnections);
    return mbBad;
}

void KeyFrame::EraseConnection(KeyFrame* pKF)
{
    bool bUpdate = false;
    {
        std::unique_lock<mutex> lock(mMutexConnections);
        if(mConnectedKeyFrameWeights.count(pKF))
        {
            mConnectedKeyFrameWeights.erase(pKF);
            bUpdate=true;
        }
    }

    if(bUpdate)
        UpdateBestCovisibles();
}

vector<size_t> KeyFrame::GetFeaturesInArea(const float &x, const float &y, const float &r) const
{
    vector<size_t> vIndices;
    vIndices.reserve(N);

    const int nMinCellX = max(0,(int)floor((x-mnMinX-r)*mfGridElementWidthInv));
    if(nMinCellX>=mnGridCols)
        return vIndices;

    const int nMaxCellX = min((int)mnGridCols-1,(int)ceil((x-mnMinX+r)*mfGridElementWidthInv));
    if(nMaxCellX<0)
        return vIndices;

    const int nMinCellY = max(0,(int)floor((y-mnMinY-r)*mfGridElementHeightInv));
    if(nMinCellY>=mnGridRows)
        return vIndices;

    const int nMaxCellY = min((int)mnGridRows-1,(int)ceil((y-mnMinY+r)*mfGridElementHeightInv));
    if(nMaxCellY<0)
        return vIndices;

    for(int ix = nMinCellX; ix<=nMaxCellX; ix++)
    {
        for(int iy = nMinCellY; iy<=nMaxCellY; iy++)
        {
            const vector<size_t> vCell = mGrid[ix][iy];
            for(size_t j=0, jend=vCell.size(); j<jend; j++)
            {
                const cv::KeyPoint &kpUn = mvKeysUn[vCell[j]];
                const float distx = kpUn.pt.x-x;
                const float disty = kpUn.pt.y-y;

                if(fabs(distx)<r && fabs(disty)<r)
                    vIndices.push_back(vCell[j]);
            }
        }
    }

    return vIndices;
}

bool KeyFrame::IsInImage(const float &x, const float &y) const
{
    return (x>=mnMinX && x<mnMaxX && y>=mnMinY && y<mnMaxY);
}

cv::Mat KeyFrame::UnprojectStereo(int i)
{
    const float z = mvDepth[i];
    if(z>0)
    {
        const float u = mvKeys[i].pt.x;
        const float v = mvKeys[i].pt.y;
        const float x = (u-cx)*z*invfx;
        const float y = (v-cy)*z*invfy;
        cv::Mat x3Dc = (cv::Mat_<float>(3,1) << x, y, z);

        std::unique_lock<mutex> lock(mMutexPose);
        return Twc.rowRange(0,3).colRange(0,3)*x3Dc+Twc.rowRange(0,3).col(3);
    }
    else
        return cv::Mat();
}

float KeyFrame::ComputeSceneMedianDepth(const int q)
{
    vector<MapPoint*> vpMapPoints;
    cv::Mat Tcw_;
    {
        std::unique_lock<mutex> lock(mMutexFeatures);
        std::unique_lock<mutex> lock2(mMutexPose);
        vpMapPoints = mvpMapPoints;
        Tcw_ = Tcw.clone();
    }

    vector<float> vDepths;
    vDepths.reserve(N);
    cv::Mat Rcw2 = Tcw_.row(2).colRange(0,3);
    Rcw2 = Rcw2.t();
    float zcw = Tcw_.at<float>(2,3);
    for(int i=0; i<N; i++)
    {
        if(mvpMapPoints[i])
        {
            MapPoint* pMP = mvpMapPoints[i];
            cv::Mat x3Dw = pMP->GetWorldPos();
            float z = Rcw2.dot(x3Dw)+zcw;
            vDepths.push_back(z);
        }
    }

    sort(vDepths.begin(),vDepths.end());

    return vDepths[(vDepths.size()-1)/q];
}

void KeyFrame::ExpandNode(MapPointersIndex* alreadyExpanded,
		MapPointersQueue* queue) const {
	CHECK_NOTNULL(alreadyExpanded);
	CHECK_NOTNULL(queue);

	alreadyExpanded->frames.AddOrDie(this);

	// Add not yet seen connections to the queue.
	for (MapPoint* mp : mvpMapPoints) {
		if (mp != nullptr) {
			InsertIfNotInIndex(mp, alreadyExpanded->points, &(queue->nodes));
		}
	}
	for (auto& connectedFrame : mConnectedKeyFrameWeights) {
		if (connectedFrame.first != nullptr) {
			InsertIfNotInIndex(connectedFrame.first, alreadyExpanded->frames,
					&(queue->nodes));
		}
	}
	for (KeyFrame* other : mvpOrderedConnectedKeyFrames) {
		CHECK_NOTNULL(other);
		InsertIfNotInIndex(other, alreadyExpanded->frames, &(queue->nodes));
	}
	if (mpParent != nullptr) {
		InsertIfNotInIndex(mpParent, alreadyExpanded->frames, &(queue->nodes));
	}
	for (KeyFrame* other : mspChildrens) {
		CHECK_NOTNULL(other);
		InsertIfNotInIndex(other, alreadyExpanded->frames, &(queue->nodes));
	}
	for (KeyFrame* other : mspLoopEdges) {
		CHECK_NOTNULL(other);
		InsertIfNotInIndex(other, alreadyExpanded->frames, &(queue->nodes));
	}
}

void KeyFrame::RestorePointers(const SerializedKeyFrame& serializedThis,
		const IndexedNodes& pointersIndex) {
	for (const long int pointIndex : serializedThis.mvpmappoints()) {
		if (pointIndex >= 0) {
			mvpMapPoints.push_back(LookupOrDie(static_cast<size_t>(pointIndex), pointersIndex.points));
		} else {
			mvpMapPoints.push_back(nullptr);
		}
	}

	for (const auto& connectedFrame : serializedThis.mconnectedkeyframeweights()) {
		mConnectedKeyFrameWeights.insert(std::make_pair(LookupOrDie(connectedFrame.frame(), pointersIndex.frames), connectedFrame.weight()));
	}
	for (const size_t connectedFrame : serializedThis.mvporderedconnectedkeyframes()) {
		mvpOrderedConnectedKeyFrames.push_back(LookupOrDie(connectedFrame, pointersIndex.frames));
	}
	if (serializedThis.mpparent() >= 0) {
		mpParent = LookupOrDie(static_cast<size_t>(serializedThis.mpparent()), pointersIndex.frames);
	}
	for (const size_t child : serializedThis.mspchildrens()) {
		mspChildrens.insert(LookupOrDie(child, pointersIndex.frames));
	}
	for (const size_t loop : serializedThis.msploopedges()) {
		mspLoopEdges.insert(LookupOrDie(loop, pointersIndex.frames));
	}
}

SerializedKeyFrame KeyFrame::ToIndexedFrame(
		const MapPointersIndex& index) const {
	SerializedKeyFrame result;

	result.set_nnextid(nNextId);
	result.set_mnid(mnId);
	result.set_mnframeid(mnFrameId);
	result.set_mtimestamp(mTimeStamp);

	// Grid (to speed up feature matching)
	result.set_mngridcols(mnGridCols);
	result.set_mngridrows(mnGridRows);
	result.set_mfgridelementwidthinv(mfGridElementWidthInv);
	result.set_mfgridelementheightinv(mfGridElementHeightInv);

	// Variables used by the tracking
	result.set_mntrackreferenceforframe(mnTrackReferenceForFrame);
	result.set_mnfusetargetforkf(mnFuseTargetForKF);

	// Variables used by the local mapping
	result.set_mnbalocalforkf(mnBALocalForKF);
	result.set_mnbafixedforkf(mnBAFixedForKF);

	// Variables used by the keyframe database
	result.set_mnloopquery(mnLoopQuery);
	result.set_mnloopwords(mnLoopWords);
	result.set_mloopscore(mLoopScore);
	result.set_mnrelocquery(mnRelocQuery);
	result.set_mnrelocwords(mnRelocWords);
	result.set_mrelocscore(mRelocScore);

	// Variables used by loop closing
	result.set_mtcwgba(SerializeMat(mTcwGBA));
	result.set_mtcwbefgba(SerializeMat(mTcwBefGBA));
	result.set_mnbaglobalforkf(mnBAGlobalForKF);

	// Calibration parameters
	result.set_fx(fx);
	result.set_fy(fy);
	result.set_cx(cx);
	result.set_cy(cy);
	result.set_invfx(invfx);
	result.set_invfy(invfy);
	result.set_mbf(mbf);
	result.set_mb(mb);
	result.set_mthdepth(mThDepth);

	// Number of KeyPoints
	result.set_n(N);

	// KeyPoints, stereo coordinate and descriptors (all associated by an index)
	result.set_mvkeys(SerializeKeyPoints(mvKeys));
	result.set_mvkeysun(SerializeKeyPoints(mvKeysUn));
	CopyVectorToRepeatedField(mvuRight, result.mutable_mvuright());
	CopyVectorToRepeatedField(mvDepth, result.mutable_mvdepth());
	result.set_mdescriptors(SerializeMat(mDescriptors));

	//BoW
	for (const auto& bowVectorElement : mBowVec) {
		BowVectorElement* bowVectorElementProto = result.add_mbowvec();
		bowVectorElementProto->set_wordid(bowVectorElement.first);
		bowVectorElementProto->set_wordvalue(bowVectorElement.second);
	}
	for (const auto& featureVectorElement : mFeatVec) {
		FeatureVectorElement* elementProto = result.add_mfeatvec();
		elementProto->set_nodeid(featureVectorElement.first);
		CopyVectorToRepeatedField(featureVectorElement.second,
				elementProto->mutable_features());
	}

	// Pose relative to parent (this is computed when bad flag is activated)
	result.set_mtcp(SerializeMat(mTcp));

	// Scale
	result.set_mnscalelevels(mnScaleLevels);
	result.set_mfscalefactor(mfScaleFactor);
	result.set_mflogscalefactor(mfLogScaleFactor);
	CopyVectorToRepeatedField(mvScaleFactors, result.mutable_mvscalefactors());
	CopyVectorToRepeatedField(mvLevelSigma2, result.mutable_mvlevelsigma2());
	CopyVectorToRepeatedField(mvInvLevelSigma2,
			result.mutable_mvinvlevelsigma2());

	// Image bounds and calibration
	result.set_mnminx(mnMinX);
	result.set_mnminy(mnMinY);
	result.set_mnmaxx(mnMaxX);
	result.set_mnmaxy(mnMaxY);
	result.set_mk(SerializeMat(mK));

	// SE3 Pose and camera center
	result.set_tcw(SerializeMat(Tcw));
	result.set_twc(SerializeMat(Twc));
	result.set_ow(SerializeMat(Ow));

	result.set_cw(SerializeMat(Cw)); // Stereo middel point. Only for visualization

	// MapPoints associated to keypoints
	for (MapPoint* mp : mvpMapPoints) {
		if (mp != nullptr) {
			result.add_mvpmappoints(index.points.IndexOrDie(mp));
		} else {
			result.add_mvpmappoints(-1);
		}
	}

	// Grid over the image to speed up feature matching
	FillProtoFromGrid(mGrid, result.mutable_mgrid());

	for (const auto& weightedFrame : mConnectedKeyFrameWeights) {
		CHECK_NOTNULL(weightedFrame.first);
		ConnectedKeyFrameWeight* resultWeightedFrame =
				result.add_mconnectedkeyframeweights();
		resultWeightedFrame->set_frame(
				index.frames.IndexOrDie(weightedFrame.first));
		resultWeightedFrame->set_weight(weightedFrame.second);
	}
	for (KeyFrame* frame : mvpOrderedConnectedKeyFrames) {
		CHECK_NOTNULL(frame);
		result.add_mvporderedconnectedkeyframes(index.frames.IndexOrDie(frame));
	}
	CopyVectorToRepeatedField(mvOrderedWeights,
			result.mutable_mvorderedweights());

	// Spanning Tree and Loop Edges
	result.set_mbfirstconnection(mbFirstConnection);
	result.set_mpparent(
			(mpParent != nullptr) ? index.frames.IndexOrDie(mpParent) : -1);
	for (KeyFrame* frame : mspChildrens) {
		CHECK_NOTNULL(frame);
		result.add_mspchildrens(index.frames.IndexOrDie(frame));
	}
	for (KeyFrame* frame : mspLoopEdges) {
		CHECK_NOTNULL(frame);
		result.add_msploopedges(index.frames.IndexOrDie(frame));
	}

	// Bad flags
	result.set_mbnoterase(mbNotErase);
	result.set_mbtobeerased(mbToBeErased);
	result.set_mbbad(mbBad);

	result.set_mhalfbaseline(mHalfBaseline); // Only for visualization

	return result;
}

} //namespace ORB_SLAM
