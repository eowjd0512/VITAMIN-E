#ifndef FEATUREEXTRACTOR_H
#define FEATUREEXTRACTOR_H
#include<unistd.h>
#include<string>
#include<thread>
#include "opencv2/opencv.hpp"
#include <vector>
#include <list>
#include "opencv2/xfeatures2d.hpp"

using namespace cv;
using namespace std;
using namespace cv::xfeatures2d;

namespace VITAMINE{

class FeatureExtractor{
    
public:
    FeatureExtractor(){
        extractor = BriefDescriptorExtractor::create();
    }; //default constructor
    FeatureExtractor(const FeatureExtractor& rhs){}; //copy constructor
    ~FeatureExtractor(){}; //destructor 
    //TODO: smart pointer

    // Compute the features and descriptors on an image.
    // Mask is ignored in the current implementation.
    void detect( const Mat& image, std::vector<KeyPoint>& keypoints, Mat& descriptors, Mat& kappa);
    void detectORB(const Mat& image, const Mat& mask, 
    vector<KeyPoint>& keypoints,
    Mat& descriptors);

    
    
private:

    Mat curvature(Mat src);
    void local_maxima(Mat img, vector<KeyPoint>& keypoints);

    Ptr<BriefDescriptorExtractor> extractor;
};

}//namespace VITAMINE

#endif // FRAME_H
