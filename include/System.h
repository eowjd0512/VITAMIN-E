//This file includes all threads as an interface.
#ifndef SYSTEM_H
#define SYSTEM_H
#include<unistd.h>
#include<string>
#include<thread>
#include "opencv2/opencv.hpp"

#include "Tracker.h"
#include "Mapper.h"
#include "Viewer.h"
#include "Map.h"
#include "MapDrawer.h"
#include "FrameDrawer.h"

using namespace std;

namespace VITAMINE{

class Map;
class MapDrawer;
class FrameDrawer;
class Tracker;
class Mapper;
class Viewer;

class System{
        
public:
    System(const string &strSettingsFile, const bool bUseViewer = true); //default constructor
    //~System{}; //destructor 
    //TODO: smart pointer

    cv::Mat track(const cv::Mat &frame);
    void Shutdown();

private:

    // Map structure that stores the pointers to all KeyFrames and MapPoints.
    Map* mpMap;

    // Tracker. It receives a frame and computes the associated camera pose.
    Tracker* mpTracker;

    // Local Mapper. It manages the local map and performs bundle adjustment.
    Mapper* mpMapper;

    // The viewer draws the map and the current camera pose. It uses Pangolin.
    Viewer* mpViewer;

    MapDrawer* mpMapDrawer;
    FrameDrawer* mpFrameDrawer;
    
    // System threads: Local Mapping, Loop Closing, Viewer.
    // The Tracking thread "lives" in the main execution thread that creates the System object.
    std::thread* mptMapping;
    std::thread* mptViewer;

    // Reset flag
    std::mutex mMutexReset;
    bool mbReset;

    // Change mode flags
    std::mutex mMutexMode;

    // Tracking state
    int mTrackingState;

    std::mutex mMutexState;
};

}//namespace VITAMINE

#endif // SYSTEM_H