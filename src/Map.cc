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

#include "orb_slam_2/Map.h"

#include<mutex>

// Serialization for saving/loading map
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/serialization/set.hpp> // TODO delete me if you serialize set elem by elem
//#include "orb_slam_2/OpenCvMatSerialization.h" // Todo delete me
#include "orb_slam_2/OpenCvSerialization.h" // Todo delete me

namespace ORB_SLAM2
{

Map::Map():mnMaxKFid(0),mnBigChangeIdx(0)
{
}

void Map::AddKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.insert(pKF);
    if(pKF->mnId>mnMaxKFid)
        mnMaxKFid=pKF->mnId;
}

void Map::AddMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.insert(pMP);
}

void Map::EraseMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.erase(pMP);

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map::EraseKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.erase(pKF);

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map::SetReferenceMapPoints(const vector<MapPoint *> &vpMPs)
{
    unique_lock<mutex> lock(mMutexMap);
    mvpReferenceMapPoints = vpMPs;
}

void Map::InformNewBigChange()
{
    unique_lock<mutex> lock(mMutexMap);
    mnBigChangeIdx++;
}

int Map::GetLastBigChangeIdx()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnBigChangeIdx;
}

vector<KeyFrame*> Map::GetAllKeyFrames()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<KeyFrame*>(mspKeyFrames.begin(),mspKeyFrames.end());
}

vector<MapPoint*> Map::GetAllMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<MapPoint*>(mspMapPoints.begin(),mspMapPoints.end());
}

long unsigned int Map::MapPointsInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspMapPoints.size();
}

long unsigned int Map::KeyFramesInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspKeyFrames.size();
}

vector<MapPoint*> Map::GetReferenceMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return mvpReferenceMapPoints;
}

long unsigned int Map::GetMaxKFid()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnMaxKFid;
}

void Map::clear()
{
    for(set<MapPoint*>::iterator sit=mspMapPoints.begin(), send=mspMapPoints.end(); sit!=send; sit++)
        delete *sit;

    for(set<KeyFrame*>::iterator sit=mspKeyFrames.begin(), send=mspKeyFrames.end(); sit!=send; sit++)
        delete *sit;

    mspMapPoints.clear();
    mspKeyFrames.clear();
    mnMaxKFid = 0;
    mvpReferenceMapPoints.clear();
    mvpKeyFrameOrigins.clear();
}

// Explicit template instantiation
template void Map::save<boost::archive::binary_oarchive>(
    boost::archive::binary_oarchive &,
    const unsigned int) const;
template void Map::load<boost::archive::binary_iarchive>(
    boost::archive::binary_iarchive &,
    const unsigned int);

template<class Archive>
void Map::save(Archive &ar, const unsigned int version) const
{
    // test boost set serialization todo delete me
    ar & test_2;
    ar & test_open_cv_matrix;

    cout << "Map::save, version: " << version << endl;
}

template<class Archive>
void Map::load(Archive &ar, const unsigned int version)
{
    // read test data
    std::set<int> new_test_2;
    cv::Mat new_test_open_cv_matrix;
    cout << "Map::load, version: " << version << endl;

    cout << "Testing set 2 after loading" << endl;
    ar & new_test_2;
    ar & new_test_open_cv_matrix;


    for (std::set<int>::iterator it=new_test_2.begin(); it!=new_test_2.end(); ++it)
      cout << (*it) << ", ";
    cout << endl;

    cout << "new_test_open_cv_matrix:" << endl;
    cout << new_test_open_cv_matrix << endl;
}

void Map::CreateTestingSet()
{
    // create test data
    for(int i = 0; i < 10; i++) {
        test_2.insert(i);
    }

    cout << "Testing set 2 before saving" << endl;
    for (std::set<int>::iterator it=test_2.begin(); it!=test_2.end(); ++it)
      cout << (*it) << ", ";

    cout << endl;

    test_open_cv_matrix = (cv::Mat_<double>(3,3) << 55, 0, 0, 0, 28, 0, 0, 0, 33);
}

} //namespace ORB_SLAM
