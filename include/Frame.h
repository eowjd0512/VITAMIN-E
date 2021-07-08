#ifndef FRAME_H
#define FRAME_H
#include<unistd.h>
#include<string>
#include<thread>
#include<vector>
#include "opencv2/opencv.hpp"
#include "FeatureExtractor.h"
#include "MapPoint.h"

#include <mutex>
using namespace std;

namespace VITAMINE{
#define FRAME_GRID_ROWS 48
#define FRAME_GRID_COLS 64

class MapPoint;

class Frame{
    
public:
    //using Ptr = std::unique_ptr<Frame>;

    Frame(){};
    // Constructor for Monocular cameras.
    Frame(const cv::Mat &imGray, FeatureExtractor* extractor,cv::Mat &K, cv::Mat &distCoef);


    Frame(const Frame& frame); //copy constructor
    //~Frame(){}; //destructor 
    
    // Extract features on the image. 
    void ExtractFeature(const cv::Mat &im);

    // Set the camera pose.
    void SetPose(cv::Mat Tcw);

    // Computes rotation, translation and camera center matrices from the camera pose.
    void UpdatePoseMatrices();

    // MapPoint observation functions
    void AddMapPoint(MapPoint* pMP, const size_t &idx);

    void insertTrackingFeature(const cv::Point pt);
    size_t TrackingFeatureSize();

    cv::Mat GetPose();
    cv::Mat GetPoseInverse();
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
    std::vector<KeyPoint> mvKeys;
    std::vector<KeyPoint> mvKeysUn;
    std::vector<Point> mvTrackedKeys;

    //ORB key points for detecting motion
    std::vector<KeyPoint> ORBKeys;
    // Brief descriptor, each row associated to a keypoint.
    Mat mBRIEFDescriptors;

    //Curvature kappa in an image pyramid
    cv::Mat kappa;

    // MapPoints associated to keypoints, NULL pointer if no association.
    std::vector<MapPoint*> mvpMapPoints;

    // Flag to identify outlier associations.
    std::vector<bool> mvbOutlier;

    // Keypoints are assigned to cells in a grid to reduce matching complexity when projecting MapPoints.
    static float mfGridElementWidthInv;
    static float mfGridElementHeightInv;
    std::vector<std::size_t> mGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS];

    // Camera pose.
    cv::Mat mTcw;
    cv::Mat mTwc;

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

    // Compute the cell of a point for feature tracking (return false if outside the grid)
    bool PosInGrid(const cv::Point &p, int &posX, int &posY);
    bool isBad();
protected:
    // Undistort keypoints given OpenCV distortion parameters.
    // Only for the RGB-D case. Stereo must be already rectified!
    // (called in the constructor).
    void UndistortKeyPoints();

    // Computes image bounds for the undistorted image (called in the constructor).
    void ComputeImageBounds(const cv::Mat &im);

    // Assign keypoints to the grid for speed up feature matching (called in the constructor).
    void AssignFeaturesToGrid();

    // Compute the cell of a keypoint (return false if outside the grid)
    bool PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY);

    bool mbBad;
    
    // Rotation, translation and camera center
    cv::Mat mRcw;
    cv::Mat mtcw;
    cv::Mat mRwc;
    cv::Mat mOw; //==mtwc

    std::mutex mMutexPose;
    std::mutex mMutexConnections;
    std::mutex mMutexFeatures;
};

}//namespace VITAMINE

#endif // FRAME_H
