#ifndef TRACKER_H
#define TRACKER_H
#include<unistd.h>
#include<string>
#include<thread>
#include "opencv2/opencv.hpp"

#include"Viewer.h"
#include"FrameDrawer.h"
#include"Map.h"
#include"Mapper.h"
#include "MapDrawer.h"
#include "System.h"

#include <mutex>
using namespace std;

namespace VITAMINE{

class Viewer;
class FrameDrawer;
class Map;
class Mapper;
class System;

class Tracker{
    
public:
    Tracker(System* pSys, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer,
                            Map* pMap, const string &strSettingPath){}; //default constructor
    Tracker(const Tracker& rhs){}; //copy constructor
    ~Tracker(){}; //destructor 
    //TODO: smart pointer

    Tracker& operator=(const Tracker& rhs()){};//copy assignment operator

    // Preprocess the input and call Track(). Extract features and performs matching.
    const void tracking();

    void SetMapper(Mapper* pMapper);
    void SetViewer(Viewer *pViewer);

private:
    //Other Thread Pointers
    Mapper* mpMapper;

    // System
    System* mpSystem;

    //Drawers
    Viewer* mpViewer;
    FrameDrawer* mpFrameDrawer;
    MapDrawer* mpMapDrawer;

    //Map
    Map* mpMap;

    
};

}//namespace VITAMINE

#endif // TRACKER_H
