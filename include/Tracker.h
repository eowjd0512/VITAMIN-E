#ifndef TRACKER_H
#define TRACKER_H
#include <unistd.h>
#include <string>
#include <thread>
#include "opencv2/opencv.hpp"

#include "Viewer.h"
#include "FrameDrawer.h"
#include "Map.h"
#include "Mapper.h"
#include "MapDrawer.h"
#include "Frame.h"
#include "FeatureExtractor.h"
#include "Initializer.h"
#include "System.h"
#include "VitamineFunction.h"

#include <mutex>
using namespace std;

namespace VITAMINE{

class Viewer;
class FrameDrawer;
class Map;
class Mapper;
class System;
class VitamineFunction;

class Tracker{
    
public:
    Tracker(System* pSys, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer,
                            Map* pMap, const string &strSettingPath); //default constructor
    
    // Preprocess the input and call Track().
    cv::Mat GrabImageMonocular(const cv::Mat &im);

    ~Tracker(){}; //destructor 

    void SetMapper(Mapper* pMapper);
    void SetViewer(Viewer *pViewer);
    void Reset();

public:
    // Tracking states
    enum eTrackingState{
        SYSTEM_NOT_READY=-1,
        NO_IMAGES_YET=0,
        NOT_INITIALIZED=1,
        OK=2,
        LOST=3
    };

    eTrackingState mState;
    eTrackingState mLastProcessedState;

    // Current & Previous Frame
    Frame mCurrentFrame, mPrevFrame;
    cv::Mat mImGray;

    // Initialization Variables (Monocular)
    std::vector<cv::Point3f> mvIniP3D;
    Frame mInitialFrame;
    

    //vitaminE main function
    VitamineFunction* vitaFunc;


    // Initalization (only for monocular)
    Initializer* mpInitializer;

private:

    // Main tracking function. It is independent of the input sensor.
    void Track();

    void FeatureTrack();

    void UpdateMotionModel();
    
    void MonocularInitialization();

    void CreateInitialMapMonocular(const vector<bool>& vbTriangulated, const vector<unsigned int>& pt_idx);
    
    //Feature
    FeatureExtractor* mpFeatureExtractor;

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

    //Calibration matrix
    cv::Mat mK;
    cv::Mat mDistCoef;

    //Motion Model
    cv::Mat mVelocity;

    //Feature matchinig
    Ptr<DescriptorMatcher> matcher;

    //Color order (true RGB, false BGR, ignored if grayscale)
    bool mbRGB;

    //The first frame
    bool firstFrame;
};

}//namespace VITAMINE

#endif // TRACKER_H
