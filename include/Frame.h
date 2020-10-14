#ifndef FRAME_H
#define FRAME_H
#include<unistd.h>
#include<string>
#include<thread>
#include "opencv2/opencv.hpp"


namespace VITAMINE{

    class Frame{
        
    public:
        Frame(){}; //default constructor
        Frame(const Frame& rhs){}; //copy constructor
        ~Frame(){}; //destructor 
        //TODO: smart pointer

    private:
    //cv::Mat

    };

}//namespace VITAMINE

#endif // FRAME_H
