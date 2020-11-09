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
    FeatureExtractor(int _nfeatures):nfeatures(_nfeatures){
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
    int nfeatures;
    Mat curvature(Mat src);
    Mat local_maxima(Mat img);
    std::vector<cv::KeyPoint> ComputeKeyPointsOctTree(const Mat& localPoints);    
    std::vector<cv::KeyPoint> DistributeOctTree(const std::vector<cv::KeyPoint>& vToDistributeKeys, const int &minX,
                                           const int &maxX, const int &minY, const int &maxY, const int &N, const int &level);


    Ptr<BriefDescriptorExtractor> extractor;
};

}//namespace VITAMINE

#endif // FRAME_H
