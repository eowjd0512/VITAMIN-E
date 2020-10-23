#ifndef FUNCTION_H
#define FUNCTION_H
#include<unistd.h>
#include<string>
#include<thread>
#include "opencv2/opencv.hpp"
#include "Frame.h"

namespace VITAMINE{
#define FRAME_GRID_ROWS 48
#define FRAME_GRID_COLS 64

class VitamineFunction{
    
//class Frame;

public:
    VitamineFunction(){}; //default constructor
    VitamineFunction(const VitamineFunction& rhs){}; //copy constructor
    ~VitamineFunction(){}; //destructor 
    //TODO: smart pointer


    //main Function
    const void loadConsecutiveFrames(Frame* prevFrame, Frame* currentFrame);
    
    const void setPrevFrame(Frame* prevFrame);
    const void getDominantMotion(const std::vector<DMatch>& good_matches);
    const void vitaTrack();

    double p_fn(const double x, const double sigma);
    double w_fn(const double x, const double sigma);
    void hill_climb(const Mat& kappa, vector<Point>& pt1, vector<Point2d> pt1_, double lmd);


    const void setInitialFeatures(); //initial feature setting
    const void addResidualFeatures(int cntThres); //add residual features after tracking
    const void drawTrackingFeatures();
    
    struct TrackedFeature{
        Point pt;
        //vector<unsigned int> viewIdx;

        TrackedFeature(Point pt_): pt(pt_) {};
    };

    vector<TrackedFeature*> tf; 

private:

    Frame* mPrevFrame;
    Frame* mCurrentFrame;
    Mat Ab;
    Mat drawFeatureMat;


};

}//namespace VITAMINE

#endif // FUNCTION_H
