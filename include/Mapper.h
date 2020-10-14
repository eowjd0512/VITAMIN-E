#ifndef MAPPER_H
#define MAPPER_H
#include<unistd.h>
#include<string>
#include<thread>
#include "opencv2/opencv.hpp"

#include "Map.h"
#include "Tracker.h"

#include <mutex>

namespace VITAMINE{

class Tracker;
class Map;

    class Mapper{
        
public:
    Mapper(Map* pMap){}; //default constructor
    Mapper(const Mapper& rhs){}; //copy constructor
    ~Mapper(){}; //destructor 
    //TODO: smart pointer

    void SetTracker(Tracker* pTracker);

    // Main function
    void Run();

    Mapper& operator=(const Mapper& rhs()){};//copy assignment operator
private:
    Map* mpMap;
    Tracker* mpTracker;
};

}//namespace VITAMINE

#endif // MAPPER_H
