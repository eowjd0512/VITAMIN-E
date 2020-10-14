#ifndef MAPDRAWER_H
#define MAPDRAWER_H
#include<unistd.h>
#include<string>
#include<thread>
#include "opencv2/opencv.hpp"


namespace VITAMINE{

    class MapDrawer{
        
    public:
        MapDrawer(){}; //default constructor
        MapDrawer(const MapDrawer& rhs){}; //copy constructor
        ~MapDrawer(){}; //destructor 
        //TODO: smart pointer

    private:
    //cv::Mat

    };

}//namespace VITAMINE

#endif // MAPDRAWER_H
