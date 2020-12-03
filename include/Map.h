#ifndef MAP_H
#define MAP_H
#include<unistd.h>
#include<string>
#include<thread>
#include "opencv2/opencv.hpp"
#include <set>
#include "MapPoint.h"
#include "Frame.h"
#include <mutex>

namespace VITAMINE{
class MapPoint;
class Frame;
class Map{
    
public:
    Map(); //default constructor
    //Map(const Map& rhs){}; //copy constructor
    ~Map(){}; //destructor 
    //TODO: smart pointer

    void AddFrame(Frame pKF);
    void AddMapPoint(MapPoint* pMP);

    long unsigned int MapPointsInMap();

    std::vector<Frame> GetAllFrames();
    std::vector<MapPoint*> GetAllMapPoints();

    std::mutex mMutexMapUpdate;
    // This avoid that two points are created simultaneously in separate threads (id conflict)
    std::mutex mMutexPointCreation;
private:

    long unsigned int mnMaxKFid;
    std::set<MapPoint*> mspMapPoints;   
    std::vector<Frame> mspFrames;
    std::mutex mMutexMap;
};

}//namespace VITAMINE

#endif // MAP_H
