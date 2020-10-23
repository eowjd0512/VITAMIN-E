#ifndef FRAME_H
#define FRAME_H
#include<unistd.h>
#include<string>
#include<thread>
#include<vector>
#include "opencv2/opencv.hpp"
#include "FeatureExtractor.h"
#include "MapPoint.h"
using namespace std;

namespace VITAMINE{

class MapPoint;

class Frame{
    
public:
    Frame(){};
    // Constructor for Monocular cameras.
    Frame(const cv::Mat &imGray, FeatureExtractor* extractor,cv::Mat &K, cv::Mat &distCoef);


    Frame(const Frame& frame); //copy constructor
    //~Frame(){}; //destructor 
    
    // Extract features on the image. 
    void ExtractFeature(const cv::Mat &im);

public:
    // Feature extractor. The right is used only in the stereo case.
    FeatureExtractor* mpFeatureExtractor;

    // Frame timestamp.
    double mTimeStamp;

    // Calibration matrix and OpenCV distortion parameters.
    cv::Mat mK;
    static float fx;
    static float fy;
    static float cx;
    static float cy;
    static float invfx;
    static float invfy;
    cv::Mat mDistCoef;

    // Number of KeyPoints.
    int N;

    // Vector of keypoints (original for visualization) and undistorted (actually used by the system).
    // In the stereo case, mvKeysUn is redundant as images must be rectified.
    // In the RGB-D case, RGB images can be distorted.
    std::vector<KeyPoint> mvKeys;
    std::vector<KeyPoint> mvKeysUn;
    std::vector<KeyPoint> mvTrackedKeys;

    // Brief descriptor, each row associated to a keypoint.
    Mat mDescriptors, mtrackedKeyDescriptors;

    //Curvature kappa
    Mat kappa;

    // MapPoints associated to keypoints, NULL pointer if no association.
    std::vector<MapPoint*> mvpMapPoints;

    // Flag to identify outlier associations.
    std::vector<bool> mvbOutlier;

    // Keypoints are assigned to cells in a grid to reduce matching complexity when projecting MapPoints.
    //static float mfGridElementWidthInv;
    //static float mfGridElementHeightInv;
    //std::vector<std::size_t> mGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS];

    // Camera pose.
    cv::Mat mTcw;

    // Current and Next Frame id.
    static long unsigned int nNextId;
    long unsigned int mnId;

    // Undistorted Image Bounds (computed once).
    static float mnMinX;
    static float mnMaxX;
    static float mnMinY;
    static float mnMaxY;

    static bool mbInitialComputations;

    //temp
    cv::Mat img;
protected:
    // Undistort keypoints given OpenCV distortion parameters.
    // Only for the RGB-D case. Stereo must be already rectified!
    // (called in the constructor).
    void UndistortKeyPoints();

    // Computes image bounds for the undistorted image (called in the constructor).
    void ComputeImageBounds(const cv::Mat &im);

    // Rotation, translation and camera center
    cv::Mat mRcw;
    cv::Mat mtcw;
    cv::Mat mRwc;
    cv::Mat mOw; //==mtwc

    
};

}//namespace VITAMINE

#endif // FRAME_H
