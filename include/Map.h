#ifndef MAP_H
#define MAP_H
#include<unistd.h>
#include<string>
#include<thread>
#include "opencv2/opencv.hpp"


namespace VITAMINE{

    class Map{
        
    public:
        Map(){}; //default constructor
        Map(const Map& rhs){}; //copy constructor
        ~Map(){}; //destructor 
        //TODO: smart pointer

    private:
    //cv::Mat

    };

}//namespace VITAMINE

#endif // MAP_H
