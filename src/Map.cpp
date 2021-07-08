#include "Map.h"
#include<mutex>

namespace VITAMINE
{
Map::Map():mnMaxKFid(0){

}
void Map::AddFrame(Frame* pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    mspFrames.push_back(pKF);
    if(pKF->mnId>mnMaxKFid)
        mnMaxKFid=pKF->mnId;
}

void Map::AddMapPoint(MapPoint *pMP)
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    mspMapPoints.insert(pMP);
}

long unsigned int Map::MapPointsInMap()
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    return mspMapPoints.size();
}

std::vector<Frame*> Map::GetAllFrames()
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    return mspFrames;
}

std::vector<MapPoint*> Map::GetAllMapPoints()
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    return std::vector<MapPoint*>(mspMapPoints.begin(),mspMapPoints.end());
}

}//namespace VITAMINE