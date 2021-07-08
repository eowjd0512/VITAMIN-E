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

class ExtractorNode
{
public:
    ExtractorNode():bNoMore(false){}

    void DivideNode(ExtractorNode &n1, ExtractorNode &n2, ExtractorNode &n3, ExtractorNode &n4);

    std::vector<cv::KeyPoint> vKeys;
    cv::Point2i UL, UR, BL, BR;
    std::list<ExtractorNode>::iterator lit;
    bool bNoMore;
};

class FeatureExtractor{
    
public:
    FeatureExtractor();
    FeatureExtractor(const FeatureExtractor& rhs){}; //copy constructor
    ~FeatureExtractor(){}; //destructor 
    //TODO: smart pointer

    // Compute the features and descriptors on an image.
    // Mask is ignored in the current implementation.
    void detect(const Mat& img, std::vector<KeyPoint>& keypoints, Mat & kappa);
    void detectFeatsForMotion(const Mat& img, vector<KeyPoint>& keypoints, Mat& descriptors);
    //std::vector<cv::Mat> mvImagePyramid;
    
private:
    std::vector<cv::Point> pattern;

    int nfeatures;
    double scaleFactor;
    int nlevels;
    int iniThFAST;
    int minThFAST;

    std::vector<int> mnFeaturesPerLevel;

    std::vector<int> umax;

    std::vector<float> mvScaleFactor;
    std::vector<float> mvInvScaleFactor;    
    std::vector<float> mvLevelSigma2;
    std::vector<float> mvInvLevelSigma2;

    cv::Mat curvature(const cv::Mat& img);
    cv::Mat local_maxima(const cv::Mat& img);
    void ComputePyramid(const cv::Mat& img);
    
    //Ptr<BriefDescriptorExtractor> extractor;
    Ptr<ORB> extractor;
};

}//namespace VITAMINE

#endif // FRAME_H
