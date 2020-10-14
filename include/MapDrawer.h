#ifndef MAPDRAWER_H
#define MAPDRAWER_H
#include<unistd.h>
#include<string>
#include<thread>
#include "opencv2/opencv.hpp"
#include<pangolin/pangolin.h>
using namespace std;

namespace VITAMINE{

    class MapDrawer{
        
    public:
        MapDrawer(Map* pMap, const string &strSettingPath){}; //default constructor

        Map* mpMap;

        MapDrawer(const MapDrawer& rhs){}; //copy constructor
        ~MapDrawer(){}; //destructor 
        //TODO: smart pointer

    private:
    //cv::Mat

    };

}//namespace VITAMINE

#endif // MAPDRAWER_H
