//This file includes all threads as an interface.
#ifndef SYSTEM_H
#define SYSTEM_H
#include<unistd.h>
#include<string>
#include<thread>
#include "opencv2/opencv.hpp"
#include "Map.h"
#include "MapDrawer.h"
#include "FrameDrawer.h"

using namespace std;

namespace VITAMINE{

class Map;
class MapDrawer;
class FrameDrawer;

    class System{
        
    public:
        System(const string &strSettingsFile, const bool bUseViewer = true); //default constructor
        //~System{}; //destructor 
        //TODO: smart pointer

        cv::Mat track(const cv::Mat &frame);
        void Shutdown();

    private:

    Map* mpMap;
    MapDrawer* mpMapDrawer;
    FrameDrawer* mpFrameDrawer;
    
    };

}//namespace VITAMINE

#endif // SYSTEM_H