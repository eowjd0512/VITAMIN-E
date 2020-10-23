#include "Tracker.h"

namespace VITAMINE
{
    Tracker::Tracker(System* pSys, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer,
                            Map* pMap, const string &strSettingPath):
    mState(NO_IMAGES_YET), mpInitializer(static_cast<Initializer*>(NULL)), mpSystem(pSys), mpViewer(NULL),
    mpFrameDrawer(pFrameDrawer), mpMapDrawer(pMapDrawer), mpMap(pMap), firstFrame(true)
    {
        // Load camera parameters from settings file

        cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
        float fx = fSettings["Camera.fx"];
        float fy = fSettings["Camera.fy"];
        float cx = fSettings["Camera.cx"];
        float cy = fSettings["Camera.cy"];
        
        cv::Mat K = cv::Mat::eye(3,3,CV_32F);
        K.at<float>(0,0) = fx;
        K.at<float>(1,1) = fy;
        K.at<float>(0,2) = cx;
        K.at<float>(1,2) = cy;
        K.copyTo(mK);

        cv::Mat DistCoef(4,1,CV_32F);
        DistCoef.at<float>(0) = fSettings["Camera.k1"];
        DistCoef.at<float>(1) = fSettings["Camera.k2"];
        DistCoef.at<float>(2) = fSettings["Camera.p1"];
        DistCoef.at<float>(3) = fSettings["Camera.p2"];
        const float k3 = fSettings["Camera.k3"];
        if(k3!=0)
        {
            DistCoef.resize(5);
            DistCoef.at<float>(4) = k3;
        }
        DistCoef.copyTo(mDistCoef);

        float fps = fSettings["Camera.fps"];
        if(fps==0)
            fps=30;

        cout << endl << "Camera Parameters: " << endl;
        cout << "- fx: " << fx << endl;
        cout << "- fy: " << fy << endl;
        cout << "- cx: " << cx << endl;
        cout << "- cy: " << cy << endl;
        cout << "- k1: " << DistCoef.at<float>(0) << endl;
        cout << "- k2: " << DistCoef.at<float>(1) << endl;
        if(DistCoef.rows==5)
            cout << "- k3: " << DistCoef.at<float>(4) << endl;
        cout << "- p1: " << DistCoef.at<float>(2) << endl;
        cout << "- p2: " << DistCoef.at<float>(3) << endl;
        cout << "- fps: " << fps << endl;

        int nRGB = fSettings["Camera.RGB"];
        mbRGB = nRGB;
        
        if(mbRGB)
            cout << "- color order: RGB (ignored if grayscale)" << endl;
        else
            cout << "- color order: BGR (ignored if grayscale)" << endl;

        // Load feature extraction parameters

        /* int nFeatures = fSettings["ORBextractor.nFeatures"];
        float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
        int nLevels = fSettings["ORBextractor.nLevels"];
        int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
        int fMinThFAST = fSettings["ORBextractor.minThFAST"]; */

        mpFeatureExtractor = new FeatureExtractor();
        vitaFunc = new VitamineFunction();
        matcher = DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);
        /* cout << endl  << "Feature Extractor Parameters: " << endl;
        cout << "- Number of Features: " << nFeatures << endl;
        cout << "- Scale Levels: " << nLevels << endl;
        cout << "- Scale Factor: " << fScaleFactor << endl;
        cout << "- Initial Fast Threshold: " << fIniThFAST << endl;
        cout << "- Minimum Fast Threshold: " << fMinThFAST << endl; */

    }

    cv::Mat Tracker::GrabImageMonocular(const cv::Mat &im)
    {
        mImGray = im;

        if(mImGray.channels()==3)
        {
            if(mbRGB)
                cvtColor(mImGray,mImGray,cv::COLOR_RGB2GRAY);
            else
                cvtColor(mImGray,mImGray,cv::COLOR_BGR2GRAY);
        }
        else if(mImGray.channels()==4)
        {
            if(mbRGB)
                cvtColor(mImGray,mImGray,cv::COLOR_RGBA2GRAY);
            else
                cvtColor(mImGray,mImGray,cv::COLOR_BGRA2GRAY);
        }

        //generate frame
        // detect feature every frame
        mCurrentFrame = Frame(mImGray,mpFeatureExtractor,mK,mDistCoef);
        Track();

        return mCurrentFrame.mTcw.clone();
    }

    void Tracker::Track()
    {
        //TODO: return if the first frame
        if (firstFrame){
            mPrevFrame = Frame(mCurrentFrame);
            firstFrame = false;
            return;
        }
        if(mState==NO_IMAGES_YET)
        {
            mState = NOT_INITIALIZED;
        }
    
        //TODO:feature track
        FeatureTrack();
        
        // Get Map Mutex -> Map cannot be changed
        /*unique_lock<mutex> lock(mpMap->mMutexMapUpdate);

         if(mState==NOT_INITIALIZED)
        {

            MonocularInitialization();

            mpFrameDrawer->Update(this);
            mPrevFrame = Frame(mCurrentFrame);

            if(mState!=OK)
                return;
        }
        else
        {
            // System is initialized. Track Frame.
            bool bOK;

            // Initial camera pose estimation using motion model or relocalization (if tracking is lost)
            // Local Mapping is activated. This is the normal behaviour, unless
            // you explicitly activate the "only tracking" mode.

            if(mState==OK)
            {
                // Local Mapping might have changed some MapPoints tracked in last frame
                CheckReplacedInLastFrame();

                bOK = TrackWithMotionModel();
                if(!bOK)
                    bOK = TrackReferenceKeyFrame();
            }
            
            // If we have an initial estimation of the camera pose and matching. Track the local map.
            if(bOK)
                bOK = TrackLocalMap();
            
            if(bOK)
                mState = OK;
            else
                mState=LOST;

            // Update drawer
            mpFrameDrawer->Update(this);

            // If tracking were good, check if we insert a keyframe
            if(bOK)
            {
                UpdateMotionModel();
                mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);

                // Clean VO matches
                for(int i=0; i<mCurrentFrame.N; i++)
                {
                    MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                    if(pMP)
                        if(pMP->Observations()<1)
                        {
                            mCurrentFrame.mvbOutlier[i] = false;
                            mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                        }
                }

                // Delete temporal MapPoints
                for(list<MapPoint*>::iterator lit = mlpTemporalPoints.begin(), lend =  mlpTemporalPoints.end(); lit!=lend; lit++)
                {
                    MapPoint* pMP = *lit;
                    delete pMP;
                }
                mlpTemporalPoints.clear();

                // Check if we need to insert a new keyframe
                if(NeedNewKeyFrame())
                    CreateNewKeyFrame();

                // We allow points with high innovation (considererd outliers by the Huber Function)
                // pass to the new keyframe, so that bundle adjustment will finally decide
                // if they are outliers or not. We don't want next frame to estimate its position
                // with those points so we discard them in the frame.
                for(int i=0; i<mCurrentFrame.N;i++)
                {
                    if(mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i])
                        mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                }
            }

            // Reset if the camera get lost soon after initialization
            if(mState==LOST)
            {
                if(mpMap->KeyFramesInMap()<=5)
                {
                    cout << "Track lost soon after initialisation, reseting..." << endl;
                    mpSystem->Reset();
                    return;
                }
            }

            //if(!mCurrentFrame.mpReferenceKF)
            //    mCurrentFrame.mpReferenceKF = mpReferenceKF;

            
        } */

        mPrevFrame = Frame(mCurrentFrame);

        imshow("mImGray", mImGray);
        cv::waitKey(30);
    }
    void Tracker::UpdateMotionModel(){
        // Update motion model
        /* if(!mLastFrame.mTcw.empty())
        {
            cv::Mat LastTwc = cv::Mat::eye(4,4,CV_32F);
            mLastFrame.GetRotationInverse().copyTo(LastTwc.rowRange(0,3).colRange(0,3));
            mLastFrame.GetCameraCenter().copyTo(LastTwc.rowRange(0,3).col(3));
            mVelocity = mCurrentFrame.mTcw*LastTwc;
        }
        else
            mVelocity = cv::Mat(); */
    }
    void Tracker::FeatureTrack(){
        //TODO:
        std::vector< std::vector<DMatch> > knn_matches;
        //cout<<mCurrentFrame.mDescriptors<<endl;
        if(mPrevFrame.mDescriptors.type()!=CV_32F) {
            mPrevFrame.mDescriptors.convertTo(mPrevFrame.mDescriptors, CV_32F);
        }

        if(mCurrentFrame.mDescriptors.type()!=CV_32F) {
            mCurrentFrame.mDescriptors.convertTo(mCurrentFrame.mDescriptors, CV_32F);
        }
        matcher->knnMatch( mPrevFrame.mDescriptors, mCurrentFrame.mDescriptors, knn_matches, 2 );

        //-- Filter matches using the Lowe's ratio test
        const float ratio_thresh = 0.7f;
        std::vector<DMatch> good_matches;
        for (size_t i = 0; i < knn_matches.size(); i++)
        {
            if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance)
            {
                good_matches.push_back(knn_matches[i][0]);
            }
        }
        
        /* //-- Draw matches
        Mat img_matches;
        drawMatches( mPrevFrame.img, mPrevFrame.mvKeys, mCurrentFrame.img, mCurrentFrame.mvKeys, good_matches, img_matches, Scalar::all(-1),
                    Scalar::all(-1), std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
        //-- Show detected matches
        imshow("Good Matches", img_matches ); 
        waitKey(30); */
        
        vitaFunc->loadConsecutiveFrames(&mPrevFrame, &mCurrentFrame);
        vitaFunc->getDominantMotion(good_matches);
        vitaFunc->vitaTrack();
        vitaFunc->addResidualFeatures(2);
        vitaFunc->drawTrackingFeatures();
        
    }
    
    void Tracker::SetMapper(Mapper *pMapper)
    {
        mpMapper=pMapper;
    }

    void Tracker::SetViewer(Viewer *pViewer)
    {
        mpViewer=pViewer;
    }

    void Tracker::Reset()
    {

        /* cout << "System Reseting" << endl;
        if(mpViewer)
        {
            mpViewer->RequestStop();
            while(!mpViewer->isStopped())
                usleep(3000);
        }

        // Reset Local Mapping
        cout << "Reseting Local Mapper...";
        mpLocalMapper->RequestReset();
        cout << " done" << endl;

        // Clear Map (this erase MapPoints and KeyFrames)
        mpMap->clear();

        Frame::nNextId = 0;
        mState = NO_IMAGES_YET;

        if(mpInitializer)
        {
            delete mpInitializer;
            mpInitializer = static_cast<Initializer*>(NULL);
        }

        if(mpViewer)
            mpViewer->Release(); */
    }
}