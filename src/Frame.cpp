#include "Frame.h"

namespace VITAMINE
{

long unsigned int Frame::nNextId=0;
bool Frame::mbInitialComputations=true;
float Frame::cx, Frame::cy, Frame::fx, Frame::fy, Frame::invfx, Frame::invfy;
float Frame::mnMinX, Frame::mnMinY, Frame::mnMaxX, Frame::mnMaxY;

//Copy Constructor
Frame::Frame(const Frame &frame)
    :mpFeatureExtractor(frame.mpFeatureExtractor),
     mK(frame.mK.clone()), mDistCoef(frame.mDistCoef.clone()),
     N(frame.N), mvKeys(frame.mvKeys),
     mvKeysUn(frame.mvKeysUn), mvTrackedKeys(frame.mvTrackedKeys),
     mDescriptors(frame.mDescriptors.clone()), mtrackedKeyDescriptors(frame.mtrackedKeyDescriptors.clone()),
     kappa(frame.kappa),
     mvpMapPoints(frame.mvpMapPoints), mvbOutlier(frame.mvbOutlier), mnId(frame.mnId),
     img(frame.img.clone())
{
    /* for(int i=0;i<FRAME_GRID_COLS;i++)
        for(int j=0; j<FRAME_GRID_ROWS; j++)
            mGrid[i][j]=frame.mGrid[i][j]; */

    /* if(!frame.mTcw.empty())
        SetPose(frame.mTcw); */
}

Frame::Frame(const cv::Mat &imGray, FeatureExtractor* extractor, cv::Mat &K, cv::Mat &distCoef)
    :mpFeatureExtractor(extractor), mK(K.clone()), mDistCoef(distCoef.clone()), img(imGray.clone())
{
    // Frame ID
    mnId=nNextId++;

    // Scale Level Info
    /* mnScaleLevels = mpFeatureExtractor->GetLevels();
    mfScaleFactor = mpFeatureExtractor->GetScaleFactor();
    mfLogScaleFactor = log(mfScaleFactor);
    mvScaleFactors = mpFeatureExtractor->GetScaleFactors();
    mvInvScaleFactors = mpFeatureExtractor->GetInverseScaleFactors();
    mvLevelSigma2 = mpFeatureExtractor->GetScaleSigmaSquares();
    mvInvLevelSigma2 = mpFeatureExtractor->GetInverseScaleSigmaSquares(); */

    // Feature extraction
    ExtractFeature(imGray);
    
    N = mvKeys.size();

    if(mvKeys.empty())
        return;

    UndistortKeyPoints();
    
    mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));
    mvbOutlier = vector<bool>(N,false);

    // This is done only for the first Frame (or after a change in the calibration)
    if(mbInitialComputations)
    {
        mvTrackedKeys = mvKeysUn;
        mtrackedKeyDescriptors = mDescriptors;
        ComputeImageBounds(imGray);

        //mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/static_cast<float>(mnMaxX-mnMinX);
        //mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/static_cast<float>(mnMaxY-mnMinY);

        fx = K.at<float>(0,0);
        fy = K.at<float>(1,1);
        cx = K.at<float>(0,2);
        cy = K.at<float>(1,2);
        invfx = 1.0f/fx;
        invfy = 1.0f/fy;

        mbInitialComputations=false;
    }

    //AssignFeaturesToGrid();

    

}

void Frame::ExtractFeature(const cv::Mat &im)
{
    mpFeatureExtractor->detect(im, mvKeys, mDescriptors, kappa);
}

/* void Frame::AssignFeaturesToGrid()
{
    int nReserve = 0.5f*N/(FRAME_GRID_COLS*FRAME_GRID_ROWS);
    for(unsigned int i=0; i<FRAME_GRID_COLS;i++)
        for (unsigned int j=0; j<FRAME_GRID_ROWS;j++)
            mGrid[i][j].reserve(nReserve);

    for(int i=0;i<N;i++)
    {
        const cv::KeyPoint &kp = mvKeysUn[i];

        int nGridPosX, nGridPosY;
        if(PosInGrid(kp,nGridPosX,nGridPosY))
            mGrid[nGridPosX][nGridPosY].push_back(i);
    }
} */

void Frame::UndistortKeyPoints()
{
    if(mDistCoef.at<float>(0)==0.0)
    {
        mvKeysUn=mvKeys;
        return;
    }

    // Fill matrix with points
    cv::Mat mat(N,2,CV_32F);
    for(int i=0; i<N; i++)
    {
        mat.at<float>(i,0)=mvKeys[i].pt.x;
        mat.at<float>(i,1)=mvKeys[i].pt.y;
    }

    // Undistort points
    mat=mat.reshape(2);
    cv::undistortPoints(mat,mat,mK,mDistCoef,cv::Mat(),mK);
    mat=mat.reshape(1);

    // Fill undistorted keypoint vector
    mvKeysUn.resize(N);
    for(int i=0; i<N; i++)
    {
        cv::KeyPoint kp = mvKeys[i];
        kp.pt.x=mat.at<float>(i,0);
        kp.pt.y=mat.at<float>(i,1);
        mvKeysUn[i]=kp;
    }
}

void Frame::ComputeImageBounds(const cv::Mat &imLeft)
{
    if(mDistCoef.at<float>(0)!=0.0)
    {
        cv::Mat mat(4,2,CV_32F);
        mat.at<float>(0,0)=0.0; mat.at<float>(0,1)=0.0;
        mat.at<float>(1,0)=imLeft.cols; mat.at<float>(1,1)=0.0;
        mat.at<float>(2,0)=0.0; mat.at<float>(2,1)=imLeft.rows;
        mat.at<float>(3,0)=imLeft.cols; mat.at<float>(3,1)=imLeft.rows;

        // Undistort corners
        mat=mat.reshape(2);
        cv::undistortPoints(mat,mat,mK,mDistCoef,cv::Mat(),mK);
        mat=mat.reshape(1);

        mnMinX = min(mat.at<float>(0,0),mat.at<float>(2,0));
        mnMaxX = max(mat.at<float>(1,0),mat.at<float>(3,0));
        mnMinY = min(mat.at<float>(0,1),mat.at<float>(1,1));
        mnMaxY = max(mat.at<float>(2,1),mat.at<float>(3,1));

    }
    else
    {
        mnMinX = 0.0f;
        mnMaxX = imLeft.cols;
        mnMinY = 0.0f;
        mnMaxY = imLeft.rows;
    }
}

}//namespace VITAMINE