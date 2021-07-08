#ifndef FEATURE_H
#define FEATURE_H
#include<unistd.h>
#include<string>
#include<thread>
#include "opencv2/opencv.hpp"
#include <vector>
#include <list>
#include "MapPoint.h"
using namespace cv;
using namespace std;

namespace VITAMINE{

class MapPoint;

class Feature{
    
public:
    Feature(std::pair<cv::Point, size_t> pt_, unsigned long int id): pt(pt_.first), color(Scalar::all(-1)) 
    {
        viewIdx.push_back(id);
        pt_history[id] = pt_;
        mapPoint = static_cast<MapPoint*>(NULL);
    }; //default constructor


    //Feature(const Feature& rhs){}; //copy constructor
    //~Feature(){}; //destructor 
    //TODO: smart pointer

    Point pt;
    Scalar color;
    map<unsigned long int, std::pair<cv::Point, size_t>> pt_history; //frame id, 2D projected point with its indx
    vector<unsigned long int> viewIdx;
  

    Mat descriptor;
    MapPoint* mapPoint;
private:

};

}//namespace VITAMINE

#endif // FRAME_H
