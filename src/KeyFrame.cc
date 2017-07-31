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

#include "orb_slam_2/KeyFrame.h"
#include "orb_slam_2/Converter.h"
#include "orb_slam_2/ORBmatcher.h"
#include<mutex>

// Serialization to save/load keyframe
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/serialization/vector.hpp>
#include "orb_slam_2/OpenCvSerialization.h"

namespace ORB_SLAM2
{

long unsigned int KeyFrame::nNextId=0;

KeyFrame::KeyFrame(Frame &F, Map *pMap, KeyFrameDatabase *pKFDB):
    mnFrameId(F.mnId),  mTimeStamp(F.mTimeStamp), mnGridCols(FRAME_GRID_COLS), mnGridRows(FRAME_GRID_ROWS),
    mfGridElementWidthInv(F.mfGridElementWidthInv), mfGridElementHeightInv(F.mfGridElementHeightInv),
    mnTrackReferenceForFrame(0), mnFuseTargetForKF(0), mnBALocalForKF(0), mnBAFixedForKF(0),
    mnLoopQuery(0), mnLoopWords(0), mnRelocQuery(0), mnRelocWords(0), mnBAGlobalForKF(0),
    fx(F.fx), fy(F.fy), cx(F.cx), cy(F.cy), invfx(F.invfx), invfy(F.invfy),
    mbf(F.mbf), mb(F.mb), mThDepth(F.mThDepth), N(F.N), mvKeys(F.mvKeys), mvKeysUn(F.mvKeysUn),
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

KeyFrame::KeyFrame():
    mnFrameId(0),  mTimeStamp(0.0), mnGridCols(FRAME_GRID_COLS), mnGridRows(FRAME_GRID_ROWS),
    mfGridElementWidthInv(0.0), mfGridElementHeightInv(0.0),
    mnTrackReferenceForFrame(0), mnFuseTargetForKF(0), mnBALocalForKF(0), mnBAFixedForKF(0),
    mnLoopQuery(0), mnLoopWords(0), mnRelocQuery(0), mnRelocWords(0), mnBAGlobalForKF(0),
    fx(0.0), fy(0.0), cx(0.0), cy(0.0), invfx(0.0), invfy(0.0),
    mbf(0.0), mb(0.0), mThDepth(0.0), N(0), mnScaleLevels(0), mfScaleFactor(0),
    mfLogScaleFactor(0.0),
    mnMinX(0), mnMinY(0), mnMaxX(0),
    mnMaxY(0)
{
    // Default Constructor to allow class serialization load/save
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
    unique_lock<mutex> lock(mMutexPose);
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
    unique_lock<mutex> lock(mMutexPose);
    return Tcw.clone();
}

cv::Mat KeyFrame::GetPoseInverse()
{
    unique_lock<mutex> lock(mMutexPose);
    return Twc.clone();
}

cv::Mat KeyFrame::GetCameraCenter()
{
    unique_lock<mutex> lock(mMutexPose);
    return Ow.clone();
}

cv::Mat KeyFrame::GetStereoCenter()
{
    unique_lock<mutex> lock(mMutexPose);
    return Cw.clone();
}


cv::Mat KeyFrame::GetRotation()
{
    unique_lock<mutex> lock(mMutexPose);
    return Tcw.rowRange(0,3).colRange(0,3).clone();
}

cv::Mat KeyFrame::GetTranslation()
{
    unique_lock<mutex> lock(mMutexPose);
    return Tcw.rowRange(0,3).col(3).clone();
}

void KeyFrame::AddConnection(KeyFrame *pKF, const int &weight)
{
    {
        unique_lock<mutex> lock(mMutexConnections);
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
    unique_lock<mutex> lock(mMutexConnections);
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
    unique_lock<mutex> lock(mMutexConnections);
    set<KeyFrame*> s;
    for(map<KeyFrame*,int>::iterator mit=mConnectedKeyFrameWeights.begin();mit!=mConnectedKeyFrameWeights.end();mit++)
        s.insert(mit->first);
    return s;
}

vector<KeyFrame*> KeyFrame::GetVectorCovisibleKeyFrames()
{
    unique_lock<mutex> lock(mMutexConnections);
    return mvpOrderedConnectedKeyFrames;
}

vector<KeyFrame*> KeyFrame::GetBestCovisibilityKeyFrames(const int &N)
{
    unique_lock<mutex> lock(mMutexConnections);
    if((int)mvpOrderedConnectedKeyFrames.size()<N)
        return mvpOrderedConnectedKeyFrames;
    else
        return vector<KeyFrame*>(mvpOrderedConnectedKeyFrames.begin(),mvpOrderedConnectedKeyFrames.begin()+N);

}

vector<KeyFrame*> KeyFrame::GetCovisiblesByWeight(const int &w)
{
    unique_lock<mutex> lock(mMutexConnections);

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
    unique_lock<mutex> lock(mMutexConnections);
    if(mConnectedKeyFrameWeights.count(pKF))
        return mConnectedKeyFrameWeights[pKF];
    else
        return 0;
}

void KeyFrame::AddMapPoint(MapPoint *pMP, const size_t &idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mvpMapPoints[idx]=pMP;
}

void KeyFrame::EraseMapPointMatch(const size_t &idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
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
    unique_lock<mutex> lock(mMutexFeatures);
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
    unique_lock<mutex> lock(mMutexFeatures);

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
    unique_lock<mutex> lock(mMutexFeatures);
    return mvpMapPoints;
}

MapPoint* KeyFrame::GetMapPoint(const size_t &idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mvpMapPoints[idx];
}

void KeyFrame::UpdateConnections()
{
    map<KeyFrame*,int> KFcounter;

    vector<MapPoint*> vpMP;

    {
        unique_lock<mutex> lockMPs(mMutexFeatures);
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
        unique_lock<mutex> lockCon(mMutexConnections);

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
    unique_lock<mutex> lockCon(mMutexConnections);
    mspChildrens.insert(pKF);
}

void KeyFrame::EraseChild(KeyFrame *pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    mspChildrens.erase(pKF);
}

void KeyFrame::ChangeParent(KeyFrame *pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    mpParent = pKF;
    pKF->AddChild(this);
}

set<KeyFrame*> KeyFrame::GetChilds()
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mspChildrens;
}

KeyFrame* KeyFrame::GetParent()
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mpParent;
}

bool KeyFrame::hasChild(KeyFrame *pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mspChildrens.count(pKF);
}

void KeyFrame::AddLoopEdge(KeyFrame *pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    mbNotErase = true;
    mspLoopEdges.insert(pKF);
}

set<KeyFrame*> KeyFrame::GetLoopEdges()
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mspLoopEdges;
}

void KeyFrame::SetNotErase()
{
    unique_lock<mutex> lock(mMutexConnections);
    mbNotErase = true;
}

void KeyFrame::SetErase()
{
    {
        unique_lock<mutex> lock(mMutexConnections);
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
        unique_lock<mutex> lock(mMutexConnections);
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
        unique_lock<mutex> lock(mMutexConnections);
        unique_lock<mutex> lock1(mMutexFeatures);

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
    unique_lock<mutex> lock(mMutexConnections);
    return mbBad;
}

void KeyFrame::EraseConnection(KeyFrame* pKF)
{
    bool bUpdate = false;
    {
        unique_lock<mutex> lock(mMutexConnections);
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

        unique_lock<mutex> lock(mMutexPose);
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
        unique_lock<mutex> lock(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPose);
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

// Explicit template instantiation
template void KeyFrame::save<boost::archive::binary_oarchive>(
    boost::archive::binary_oarchive &,
    const unsigned int) const;
template void KeyFrame::load<boost::archive::binary_iarchive>(
    boost::archive::binary_iarchive &,
    const unsigned int);

template<class Archive>
void KeyFrame::save(Archive &ar, const unsigned int version) const
{
    // TODO check
    /*if (mbBad)
            return;*/
    // TODO delete me
    cout << "KeyFrame::save." << endl;

    ar & nNextId;
    ar & mnId;
    ar & mnFrameId;

    ar & mTimeStamp;

    ar & mnGridCols;
    ar & mnGridRows;
    ar & mfGridElementWidthInv;
    ar & mfGridElementHeightInv;

    ar & mnTrackReferenceForFrame;
    ar & mnFuseTargetForKF;

    ar & mnBALocalForKF;
    ar & mnBAFixedForKF;

    ar & mnLoopQuery;
    ar & mnLoopWords;
    ar & mLoopScore;
    ar & mnRelocQuery;
    ar & mnRelocWords;
    ar & mRelocScore;

    ar & mTcwGBA;
    ar & mTcwBefGBA;
    ar & mnBAGlobalForKF;

    ar & fx;
    ar & fy;
    ar & cx;
    ar & cy;
    ar & invfx;
    ar & invfy;
    ar & mbf;
    ar & mb;
    ar & mThDepth;

    ar & N;

    ar & mvKeys;
    ar & mvKeysUn;
    ar & mvuRight;
    ar & mvDepth;
    ar & mDescriptors;

    // TODO (marco-tranzatto) What about BoW variables?
    // mBowVec
    // mFeatVec

    ar & mTcp;

    ar & mnScaleLevels;
    ar & mfScaleFactor;
    ar & mfLogScaleFactor;
    ar & mvScaleFactors;
    ar & mvLevelSigma2;
    ar & mvInvLevelSigma2;

    ar & mnMinX;
    ar & mnMinY;
    ar & mnMaxX;
    ar & mnMaxY;
    ar & mK;

    ar & Tcw;
    ar & Twc;
    ar & Ow;

    ar & Cw;

    // mvpMapPoints: save each map point id
    SerializerSaveId(ar, mvpMapPoints);

    // TODO (marco-tranzatto) What about BoW variables?
    // mpKeyFrameDB
    // mpORBvocabulary

    ar & mGrid;

    // mConnectedKeyFrameWeights: save id and weight
    SerializerSaveConnectedKeyFrameWeights(ar, mConnectedKeyFrameWeights);
    // mvpOrderedConnectedKeyFrames: save id
    SerializerSaveId(ar, mvpOrderedConnectedKeyFrames);
    ar & mvOrderedWeights;

    ar & mbFirstConnection;
    // mpParent
    SerializerSaveParent(ar, mpParent);
    // mspChildrens: save id
    SerializerSaveId(ar, mspChildrens);
    // mspLoopEdges: save id
    SerializerSaveId(ar, mspLoopEdges);

    ar & mbNotErase;
    ar & mbToBeErased;
    ar & mbBad;

    ar & mHalfBaseline;

    // TODO and mpMap?!

    // TODO delete me
    unsigned int test_data = 55;
    ar & test_data;

    // TODO delete me
    cout << "End KeyFrame::save." << endl;
}

template<class Archive>
void KeyFrame::load(Archive &ar, const unsigned int version)
{
    // TODO delete me
    cout << "KeyFrame::load." << endl;

    int nItems;bool is_id = false;
    bool has_parent = false;
    long unsigned int t_nId;
    int ConKfWeight = 0;

    ar & nNextId;
    ar & mnId;
    ar & const_cast<long unsigned int &> (mnFrameId);

    ar & const_cast<double &> (mTimeStamp);

    ar & const_cast<int &> (mnGridCols);
    ar & const_cast<int &> (mnGridRows);
    ar & const_cast<float &> (mfGridElementWidthInv);
    ar & const_cast<float &> (mfGridElementHeightInv);

    ar & mnTrackReferenceForFrame;
    ar & mnFuseTargetForKF;

    ar & mnBALocalForKF;
    ar & mnBAFixedForKF;

    ar & mnLoopQuery;
    ar & mnLoopWords;
    ar & mLoopScore;
    ar & mnRelocQuery;
    ar & mnRelocWords;
    ar & mRelocScore;

    ar & mTcwGBA;
    ar & mTcwBefGBA;
    ar & mnBAGlobalForKF;

    ar & const_cast<float &> (fx);
    ar & const_cast<float &> (fy);
    ar & const_cast<float &> (cx);
    ar & const_cast<float &> (cy);
    ar & const_cast<float &> (invfx);
    ar & const_cast<float &> (invfy);
    ar & const_cast<float &> (mbf);
    ar & const_cast<float &> (mb);
    ar & const_cast<float &> (mThDepth);

    ar & const_cast<int &> (N);

    ar & const_cast<std::vector<cv::KeyPoint> &> (mvKeys);
    ar & const_cast<std::vector<cv::KeyPoint> &> (mvKeysUn);
    ar & const_cast<std::vector<float> &> (mvuRight);
    ar & const_cast<std::vector<float> &> (mvDepth);
    ar & const_cast<cv::Mat &> (mDescriptors);

    // TODO BoW?

    ar & mTcp;

    ar & const_cast<int &> (mnScaleLevels);
    ar & const_cast<float &> (mfScaleFactor);
    ar & const_cast<float &> (mfLogScaleFactor);
    ar & const_cast<std::vector<float> &> (mvScaleFactors);
    ar & const_cast<std::vector<float> &> (mvLevelSigma2);
    ar & const_cast<std::vector<float> &> (mvInvLevelSigma2);

    ar & const_cast<int &> (mnMinX);
    ar & const_cast<int &> (mnMinY);
    ar & const_cast<int &> (mnMaxX);
    ar & const_cast<int &> (mnMaxY);
    ar & const_cast<cv::Mat &> (mK);

    ar & const_cast<cv::Mat &> (Tcw);
    ar & const_cast<cv::Mat &> (Twc);
    ar & const_cast<cv::Mat &> (Ow);

    ar & const_cast<cv::Mat &> (Cw);

    // mvpMapPoints_nId
    //mvpMapPoints.resize(nItems); // TODO ncessary?
    SerializerLoadId_nId(ar, &mvpMapPoints_nId);

    // TODO BoW?
    // mpKeyFrameDB?
    // mpORBvocabulary?

    ar & mGrid;

    // mConnectedKeyFrameWeights_nId
    SerializerLoadConnectedKeyFrameWeights_nId(ar,
        &mConnectedKeyFrameWeights_nId);
    // mvpOrderedConnectedKeyFrames_nId
    SerializerLoadId_nId(ar, &mvpOrderedConnectedKeyFrames_nId);
    // Load each mvOrderedWeights
    ar & const_cast<std::vector<int> &>(mvOrderedWeights);

    ar & const_cast<bool &> (mbFirstConnection);
    // mParent_KfId_map
    SerializerLoadParent_KfId(ar, &mParent_KfId_map);
    // mspChildrens_nId
    SerializerLoadId_nId(ar, &mspChildrens_nId);
    // mspLoopEdges_nId
    SerializerLoadId_nId(ar, &mspLoopEdges_nId);

    ar & mbNotErase;
    ar & mbToBeErased;
    ar & mbBad;
    ar & mHalfBaseline;

    // TODO delete me
    unsigned int test_data ;
    ar & test_data;
    cout << "KeyFrame::load, test_data: " << test_data << endl;

    // TODO delete me
    cout << "End KeyFrame::load." << endl;
}

template<class Archive, class DataStr>
void KeyFrame::SerializerSaveId(Archive &ar, const DataStr &container) const
{
    int numItems;
    bool isId;
    long unsigned int itemId;

    numItems = container.size();
    ar & numItems;
    for (typename DataStr::const_iterator it = container.begin();
         it != container.end();
         ++it)
    {
        if (*it == NULL)
        {
            isId = false;
            ar & isId;
        }
        else
        {
            isId = true;
            ar & isId;
            itemId =  (**it).mnId;
            ar & itemId;
        }
    }
}

template<class Archive>
void KeyFrame::SerializerSaveConnectedKeyFrameWeights(Archive &ar,
    const std::map<KeyFrame*,int> &container) const
{
    int numItems;
    bool isId;
    long unsigned int itemId;
    int conKfWeight;

    numItems = container.size();
    ar & numItems;

    for(std::map<KeyFrame*,int>::const_iterator it = container.begin();
        it != container.end();
        ++it)
    {
        if(it->first == NULL)
        {
            isId = false;
            ar & isId;
        }
        else
        {
            isId = true;
            ar & isId;
            itemId =  it->first->mnId;
            ar & itemId;
            conKfWeight = it->second;
            ar & conKfWeight;
        }
    }
}

template<class Archive>
void KeyFrame::SerializerSaveParent(Archive &ar,
        const KeyFrame* const &container) const
{
    if (container)
    {
        bool hasParent = true;
        ar & hasParent;
        ar & container->mnId;
    }
    else
    {
        bool hasParent = false;
        ar & hasParent;
    }
}

template<class Archive, class DataStr>
void KeyFrame::SerializerLoadId_nId(Archive &ar, DataStr* pContainer)
{
    int numItems;
    bool isId;
    long unsigned int itemId;
    MapId mapId;

    ar & numItems;
    mvpMapPoints.resize(numItems);
    int j=0;
    for(int i = 0; i < numItems; i++)
    {
        ar & isId;
        if(isId)
        {
            j++;
            ar & itemId;
            mapId.is_valid = true;
            mapId.id= itemId;
            (*pContainer)[i] = mapId;
        }
        else
        {
            mapId.is_valid = false;
            mapId.id= 0;
            (*pContainer)[i] = mapId;
        }
    }
}

template<class Archive>
void KeyFrame::SerializerLoadConnectedKeyFrameWeights_nId(Archive &ar,
    std::map<long unsigned int, int>* pContainer)
{
    int numItems;
    bool isId;
    long unsigned int itemId;
    int conKfWeight;

    ar & numItems;
    for(int i = 0; i < numItems; ++i)
    {
        ar & isId;
        if (isId)
        {
            ar & itemId;
            ar & conKfWeight;
            (*pContainer)[itemId] = conKfWeight;
        }
    }
}

template<class Archive>
void KeyFrame::SerializerLoadParent_KfId(Archive &ar,
    MapId *pContainer)
{
    bool hasParent;
    ar & hasParent;
    if (hasParent)
    {
        pContainer->is_valid = true;
        ar & pContainer->id;
    }
    else
    {
        pContainer->is_valid = false;
        pContainer->id = 0;
    }
}


void KeyFrame::SetMapPoints(std::vector<MapPoint*> spMapPoints)
{
    // We assume the mvpMapPoints_nId list has been initialized and contains the Map point IDS
    // With nid, Search the KeyFrame List and populate mvpMapPoints
    long unsigned int id;
    bool is_valid = false;
    int j = 0;

    for(std::map<long unsigned int,MapId>::iterator it = mvpMapPoints_nId.begin();
        it != mvpMapPoints_nId.end();
        j++,++it)
    {
        is_valid = it->second.is_valid;
        if (!is_valid)
        {
            //j--;
            //continue;
            mvpMapPoints[j] = static_cast<MapPoint*>(NULL);
        }
        else
        {
            id = it->second.id;
            mvpMapPoints[j] = spMapPoints[id];
        }

   }
}

} //namespace ORB_SLAM
