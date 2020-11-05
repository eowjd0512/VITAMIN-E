#ifndef MAPPOINT_H
#define MAPPOINT_H
#include<unistd.h>
#include<string>
#include<thread>
#include "opencv2/opencv.hpp"


namespace VITAMINE{

class MapPoint{ //TODO: smart pointer
    
public:
    MapPoint(const cv::Mat &Pos); //default constructor
    //MapPoint(const MapPoint& rhs){}; //copy constructor
    ~MapPoint(){}; //destructor 
    
    bool isBad();
    cv::Mat GetWorldPos();
private:
//cv::Mat
    cv::Mat mWorldPos;
    
    // Bad flag (we do not currently erase MapPoint from memory)
     bool mbBad;
     
    std::mutex mMutexPos;
    std::mutex mMutexFeatures;
};

}//namespace VITAMINE

#endif // MAP_H
