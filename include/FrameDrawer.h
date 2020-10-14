#ifndef FRAMEDRAWER_H
#define FRAMEDRAWER_H
#include<unistd.h>
#include<string>
#include<thread>
#include "opencv2/opencv.hpp"

#include "Tracker.h"
#include "Map.h"
#include<mutex>

namespace VITAMINE{

class Tracker;
class Viewer;

    class FrameDrawer{
        
    public:
        FrameDrawer(Map* pMap){}; //default constructor
        FrameDrawer(const FrameDrawer& rhs){}; //copy constructor
        ~FrameDrawer(){}; //destructor 
        //TODO: smart pointer

    private:
    //cv::Mat
    Map* mpMap;
    };

}//namespace VITAMINE

#endif // FRAMEDRAWER_H
