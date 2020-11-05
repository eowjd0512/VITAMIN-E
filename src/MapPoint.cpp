#include "MapPoint.h"

namespace VITAMINE
{

MapPoint::MapPoint(const cv::Mat &Pos)
:mbBad(false){
    Pos.copyTo(mWorldPos);
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
}//namespace VITAMINE