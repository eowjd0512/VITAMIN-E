#ifndef MAPPOINT_H
#define MAPPOINT_H
#include<unistd.h>
#include<string>
#include<thread>
#include "opencv2/opencv.hpp"
#include "Frame.h"
#include "Map.h"
#include <mutex>

namespace VITAMINE{
class Map;

class MapPoint{ //TODO: smart pointer
    
public:
    MapPoint(const cv::Mat &Pos, Frame* RefF, Map* pMap); //default constructor
    //MapPoint(const MapPoint& rhs){}; //copy constructor
    ~MapPoint(){}; //destructor 
    
    void AddObservation(Frame* pF,size_t idx);
    std::map<Frame*,size_t> GetObservations();

    bool isBad();
    cv::Mat GetWorldPos();
    void SetWorldPos(const cv::Mat &Pos);

    static std::mutex mGlobalMutex;
    long unsigned int mnId;
    static long unsigned int nNextId;

private:
    int nObs{0};

    // Keyframes observing the point and associated index in keyframe
     std::map<Frame*,size_t> mObservations;

    cv::Mat mWorldPos;
    
    // Bad flag (we do not currently erase MapPoint from memory)
     bool mbBad;
     
     Frame* mpRefF;
     Map* mpMap;

    std::mutex mMutexPos;
    std::mutex mMutexFeatures;
};

}//namespace VITAMINE

#endif // MAP_H
