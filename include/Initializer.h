#ifndef INITIALIZER_H
#define INITIALIZER_H
#include<unistd.h>
#include<string>
#include<thread>
#include "opencv2/opencv.hpp"


namespace VITAMINE{

    class Initializer{
        
    public:
        Initializer(){}; //default constructor
        Initializer(const Initializer& rhs){}; //copy constructor
        ~Initializer(){}; //destructor 
        //TODO: smart pointer

    private:
    //cv::Mat

    };

}//namespace VITAMINE

#endif // FRAME_H
