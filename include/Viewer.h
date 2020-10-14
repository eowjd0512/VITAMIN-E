#ifndef VIEWER_H
#define VIEWER_H
#include<unistd.h>
#include<string>
#include<thread>
#include "opencv2/opencv.hpp"

#include "FrameDrawer.h"
#include "MapDrawer.h"
#include "Tracker.h"
#include "System.h"

#include <mutex>
using namespace std;

namespace VITAMINE{

class Tracker;
class FrameDrawer;
class MapDrawer;
class System;

class Viewer{
    
public:
    Viewer(System* pSystem, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Tracker *pTracking, const string &strSettingPath)
    {}; //default constructor
    Viewer(const Viewer& rhs){}; //copy constructor
    ~Viewer(){}; //destructor 
    //TODO: smart pointer

    //main function
    void Run();

    Viewer& operator=(const Viewer& rhs()){};//copy assignment operator
private:
//cv::Mat

};

}//namespace VITAMINE

#endif // VIEWER_H
