#include "MapPoint.h"
#include <mutex>
namespace VITAMINE
{
std::mutex MapPoint::mGlobalMutex;
long unsigned int MapPoint::nNextId=0;

MapPoint::MapPoint(const cv::Mat &Pos)
:mbBad(false){
    Pos.copyTo(mWorldPos);

    // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    std::unique_lock<std::mutex> lock(mpMap->mMutexPointCreation);
    mnId=nNextId++;
}

bool MapPoint::isBad()
{
    std::unique_lock<std::mutex> lock(mMutexFeatures);
    std::unique_lock<std::mutex> lock2(mMutexPos);
    return mbBad;
}

cv::Mat MapPoint::GetWorldPos()
{
    std::unique_lock<std::mutex> lock(mMutexPos);
    return mWorldPos.clone();
}

void MapPoint::SetWorldPos(const cv::Mat &Pos)
{
    std::unique_lock<std::mutex> lock2(mGlobalMutex);
    std::unique_lock<std::mutex> lock(mMutexPos);
    Pos.copyTo(mWorldPos);
}

}//namespace VITAMINE