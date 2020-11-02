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
    Viewer(System* pSystem, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Tracker *pTracking, const string &strSettingPath); //default constructor
    Viewer(const Viewer& rhs){}; //copy constructor
    ~Viewer(){}; //destructor 
    //TODO: smart pointer

        // Main thread function. Draw points, keyframes, the current camera pose and the last processed
    // frame. Drawing is refreshed according to the camera fps. We use Pangolin.
    void Run();

    void RequestFinish();

    void RequestStop();

    bool isFinished();

    bool isStopped();

    void Release();

private:

    bool Stop();

    System* mpSystem;
    FrameDrawer* mpFrameDrawer;
    MapDrawer* mpMapDrawer;
    Tracker* mpTracker;

    // 1/fps in ms
    double mT;
    float mImageWidth, mImageHeight;

    float mViewpointX, mViewpointY, mViewpointZ, mViewpointF;

    bool CheckFinish();
    void SetFinish();
    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;

    bool mbStopped;
    bool mbStopRequested;
    std::mutex mMutexStop;
};

}//namespace VITAMINE

#endif // VIEWER_H
