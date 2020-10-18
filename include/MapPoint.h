#ifndef MAPPOINT_H
#define MAPPOINT_H
#include<unistd.h>
#include<string>
#include<thread>
#include "opencv2/opencv.hpp"


namespace VITAMINE{

class MapPoint{
    
public:
    MapPoint(){}; //default constructor
    MapPoint(const MapPoint& rhs){}; //copy constructor
    ~MapPoint(){}; //destructor 
    //TODO: smart pointer

private:
//cv::Mat

};

}//namespace VITAMINE

#endif // MAP_H
