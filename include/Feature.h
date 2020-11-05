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
    Feature(Point pt_, unsigned int id): pt(pt_), color(Scalar::all(-1)) 
    {
        viewIdx.push_back(id);
        pt_history[id] = pt;
    }; //default constructor


    //Feature(const Feature& rhs){}; //copy constructor
    //~Feature(){}; //destructor 
    //TODO: smart pointer

    Point pt;
    Scalar color;
    map<unsigned int, cv::Point> pt_history;
    vector<unsigned int> viewIdx;

    Mat descriptor;
    MapPoint* mapPoint;
private:

};

}//namespace VITAMINE

#endif // FRAME_H
