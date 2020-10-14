#ifndef FRAMEDRAWER_H
#define FRAMEDRAWER_H
#include<unistd.h>
#include<string>
#include<thread>
#include "opencv2/opencv.hpp"


namespace VITAMINE{

    class FrameDrawer{
        
    public:
        FrameDrawer(){}; //default constructor
        FrameDrawer(const FrameDrawer& rhs){}; //copy constructor
        ~FrameDrawer(){}; //destructor 
        //TODO: smart pointer

    private:
    //cv::Mat

    };

}//namespace VITAMINE

#endif // FRAMEDRAWER_H
