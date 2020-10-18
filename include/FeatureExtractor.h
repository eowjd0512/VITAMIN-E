#ifndef FEATUREEXTRACTOR_H
#define FEATUREEXTRACTOR_H
#include<unistd.h>
#include<string>
#include<thread>
#include "opencv2/opencv.hpp"


namespace VITAMINE{

    class FeatureExtractor{
        
    public:
        FeatureExtractor(){}; //default constructor
        FeatureExtractor(const FeatureExtractor& rhs){}; //copy constructor
        ~FeatureExtractor(){}; //destructor 
        //TODO: smart pointer

    private:
    //cv::Mat

    };

}//namespace VITAMINE

#endif // FRAME_H
