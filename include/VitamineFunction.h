#ifndef FUNCTION_H
#define FUNCTION_H
#include<unistd.h>
#include<string>
#include<thread>
#include "opencv2/opencv.hpp"
#include "Frame.h"
#include "Feature.h"

namespace VITAMINE{
#define FRAME_GRID_ROWS 48
#define FRAME_GRID_COLS 64

class VitamineFunction{
    
//class Frame;

public:
    VitamineFunction(){}; //default constructor
    //VitamineFunction(const VitamineFunction& rhs){}; //copy constructor
    ~VitamineFunction(){}; //destructor 
    //TODO: smart pointer


    void AddMapPoint(MapPoint* pMP,const unsigned int pt_idx) noexcept;



    //main Function
    const void loadConsecutiveFrames(Frame* prevFrame, Frame* currentFrame);
    
    //const void setPrevFrame(Frame* prevFrame)noexcept;
    const void getDominantMotion(const std::vector<DMatch>& good_matches);
    const void vitaTrack();

    double p_fn(const double x, const double sigma) const;
    double w_fn(const double x, const double sigma) const;
    void hill_climb(const Mat& kappa, vector<Point>& pt1, vector<Point2d> pt1_, double lmd);

    //get tracked feature num of id-th frame
    int getTrackedFeatureNum(unsigned int frame_id) const noexcept;

    const void setInitialFeatures(); //initial feature setting
    const void addResidualFeatures(int cntThres); //add residual features after tracking
    const void drawTrackingFeatures();
 

    vector<Feature*> tf; 

private:

    Frame* mPrevFrame;
    Frame* mCurrentFrame;
    Mat Ab;
    Mat drawFeatureMat;


};

}//namespace VITAMINE

#endif // FUNCTION_H
