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
       /*  mdata, dbg = matcher.match(db[-2], db[-1])
        T_A, T_b = get_dominant_motion(mdata)
        trk, good = vitatrack(trk, kappa, T_A, T_b) */
        
        //visualize with debug
        /* path = path[:, good]
        cols = cols[good]
        path = np.append(path, trk[None,:], axis=0)

        tmp = img.copy()
        for p, c in zip(path.swapaxes(0,1)[...,::-1], cols):
            cv2.polylines(tmp,
                    #path.swapaxes(0,1)[...,::-1],
                    p[None,...],
                    False, c
                    )
        img = cv2.addWeighted(img, 0.75, tmp, 0.25, 0.0)
        for p, c in zip(trk,cols):
            cv2.circle(img, (p[1], p[0]), 2, c)


        luv = cv2.cvtColor(img, cv2.COLOR_BGR2LUV)
        luv[..., 2] = 128
        img2 = cv2.cvtColor(luv, cv2.COLOR_LUV2BGR)
        viz = np.clip(img/255.+maxima, 0.0, 1.0)
        #viz = cv2.addWeighted(img, 0.2, kappa, 255./0.8, 0.0,
        #        dtype=cv2.CV_8U)
        #viz = np.concatenate( (img, img2), axis=1)
        #viz = kappa - kappa.min(axis=(0,1),keepdims=True)
        cv2.imwrite('/tmp/frame{:04d}.png'.format(iter), (viz*255).astype(np.uint8) )
        cv2.imshow('img', viz)
        k = cv2.waitKey(1)
        if k in [27, ord('q')]:
            break
        iter += 1 */

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