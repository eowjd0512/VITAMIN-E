#include "Tracker.h"
#include <chrono>

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

        int nFeatures = fSettings["ORBextractor.nFeatures"];
        /* float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
        int nLevels = fSettings["ORBextractor.nLevels"];
        int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
        int fMinThFAST = fSettings["ORBextractor.minThFAST"]; */

        mpFeatureExtractor = new FeatureExtractor(nFeatures);
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
        mLastProcessedState=mState;
        //TODO:feature track
        FeatureTrack();
        
        // Get Map Mutex -> Map cannot be changed
        unique_lock<mutex> lock(mpMap->mMutexMapUpdate);

         if(mState==NOT_INITIALIZED)
        {
            
            MonocularInitialization();

            mpFrameDrawer->Update(this);
            mPrevFrame = Frame(mCurrentFrame);

            if(mState!=OK)
                return;
        }
        /* else
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
    }

    void Tracker::MonocularInitialization()
    {

        if(!mpInitializer)
        {
            // Set Reference Frame
            if(vitaFunc->tf.size()>100)
            {
                mInitialFrame = Frame(mCurrentFrame);
                //mLastFrame = Frame(mCurrentFrame);
                /* mvbPrevMatched.resize(mCurrentFrame.mvKeysUn.size());
                for(size_t i=0; i<mCurrentFrame.mvKeysUn.size(); i++)
                    mvbPrevMatched[i]=mCurrentFrame.mvKeysUn[i].pt; */

                if(mpInitializer)
                    delete mpInitializer;

                mpInitializer =  new Initializer(mCurrentFrame,1.0,500, vitaFunc);

                //fill(mvIniMatches.begin(),mvIniMatches.end(),-1);

                return;
            }
        }
        else
        {
            int nmatches = vitaFunc->getTrackedFeatureNum(mInitialFrame.mnId);
            
            // Try to initialize
            if(nmatches<=100)
            {
                delete mpInitializer;
                mpInitializer = static_cast<Initializer*>(NULL);
                //fill(mvIniMatches.begin(),mvIniMatches.end(),-1);
                return;
            }

            // Find correspondences
            //ORBmatcher matcher(0.9,true);
            //int nmatches = matcher.SearchForInitialization(mInitialFrame,mCurrentFrame,mvbPrevMatched,mvIniMatches,100);


            cv::Mat Rcw; // Current Camera Rotation
            cv::Mat tcw; // Current Camera Translation
            vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)
            vector<unsigned int> idx;
            if(mpInitializer->Initialize(mCurrentFrame, Rcw, tcw, mvIniP3D, vbTriangulated, idx))
            {
                cout<<"here?"<<endl;
                /* for(size_t i=0, iend=mvIniMatches.size(); i<iend;i++)
                {
                    if(mvIniMatches[i]>=0 && !vbTriangulated[i])
                    {
                        mvIniMatches[i]=-1;
                        nmatches--;
                    }
                } */

                // Set Frame Poses
                mInitialFrame.SetPose(cv::Mat::eye(4,4,CV_32F));
                cv::Mat Tcw = cv::Mat::eye(4,4,CV_32F);
                Rcw.copyTo(Tcw.rowRange(0,3).colRange(0,3));
                tcw.copyTo(Tcw.rowRange(0,3).col(3));
                mCurrentFrame.SetPose(Tcw);

                int cnt=0;
                for(auto i=0; i<vbTriangulated.size(); i++){
                    if(vbTriangulated[i])
                        cnt++;
                }
                cout<<"triangulated: "<<cnt<<endl;
                CreateInitialMapMonocular(vbTriangulated, idx);
            }
        }
    }
    void Tracker::CreateInitialMapMonocular(const vector<bool>& vbTriangulated, const vector<unsigned int>& pt_idx)
    {
        // Create KeyFrames
        //KeyFrame* pKFini = new KeyFrame(mInitialFrame,mpMap,mpKeyFrameDB);
        //KeyFrame* pKFcur = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);


        //pKFini->ComputeBoW();
        //pKFcur->ComputeBoW();

        // Insert KFs in the map
        mpMap->AddFrame(mInitialFrame);
        mpMap->AddFrame(mCurrentFrame);

        // Create MapPoints and asscoiate to keyframes
        
        float medianDepth = 0;
        for(size_t i=0; i<pt_idx.size();i++)
        {
            if(!vbTriangulated[i])
                continue;

            //Create MapPoint.
            cv::Mat worldPos(mvIniP3D[i]);
            MapPoint* pMP = new MapPoint(worldPos);
            medianDepth += worldPos.at<float>(2);
            //pKFini->AddMapPoint(pMP,i);
            //pKFcur->AddMapPoint(pMP,mvIniMatches[i]);

            //pMP->AddObservation(mInitialFrame.mnId,i);
            //pMP->AddObservation(mCurrentFrame.mnId,mvIniMatches[i]);

            //pMP->ComputeDistinctiveDescriptors();
            //pMP->UpdateNormalAndDepth();

            //Fill Current Frame structure
            //mCurrentFrame.mvpMapPoints[mvIniMatches[i]] = pMP;
            //mCurrentFrame.mvbOutlier[mvIniMatches[i]] = false;

            //Add to Map
            mpMap->AddMapPoint(pMP);
            vitaFunc->AddMapPoint(pMP,pt_idx[i]);
        }
        
        // Update Connections
        //pKFini->UpdateConnections();
        //pKFcur->UpdateConnections();

        cout<<mInitialFrame.mTcw<<endl;
        cout<<mCurrentFrame.mTcw<<endl;
        

        // Bundle Adjustment
        cout << "New Map created with " << mpMap->MapPointsInMap() << " points" << endl;
        medianDepth/=(float)mpMap->MapPointsInMap();
        //Optimizer::GlobalBundleAdjustemnt(mpMap,20);

        // Set median depth to 1
        //float medianDepth = pKFini->ComputeSceneMedianDepth(2);
        float invMedianDepth = 1.0f/medianDepth;

        if(medianDepth<0)// || pKFcur->TrackedMapPoints(1)<100)
        {
            cout << "Wrong initialization, reseting..." << endl;
            Reset();
            return;
        }

        // Scale initial baseline
         cv::Mat Tc2w = mCurrentFrame.GetPose();
        Tc2w.col(3).rowRange(0,3) = Tc2w.col(3).rowRange(0,3)*invMedianDepth;
        mCurrentFrame.SetPose(Tc2w);
        
        // Scale points
        vector<MapPoint*> vpAllMapPoints = mpMap->GetAllMapPoints();
        for(size_t iMP=0; iMP<vpAllMapPoints.size(); iMP++)
        {
            if(vpAllMapPoints[iMP])
            {
                MapPoint* pMP = vpAllMapPoints[iMP];
                pMP->SetWorldPos(pMP->GetWorldPos()*invMedianDepth);
            }
        }

        //mpLocalMapper->InsertKeyFrame(pKFini);
        //mpLocalMapper->InsertKeyFrame(pKFcur);

        //mCurrentFrame.SetPose(pKFcur->GetPose());
        //mnLastKeyFrameId=mCurrentFrame.mnId;
        //mpLastKeyFrame = pKFcur;

        //mvpLocalKeyFrames.push_back(pKFcur);
        //mvpLocalKeyFrames.push_back(pKFini);
        //mvpLocalMapPoints=mpMap->GetAllMapPoints();
        //mpReferenceKF = pKFcur;
        //mCurrentFrame.mpReferenceKF = pKFcur;

        //mLastFrame = Frame(mCurrentFrame);

        //mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

        mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.GetPose());

        //mpMap->mvpKeyFrameOrigins.push_back(pKFini);

        mState=OK;
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

        //std::chrono::system_clock::time_point start = std::chrono::system_clock::now();
        vitaFunc->getDominantMotion(good_matches);
        vitaFunc->vitaTrack();
        //std::chrono::duration<double> sec = std::chrono::system_clock::now() - start;
        //std::cout << "Tracking time : " << sec.count() << " seconds" << std::endl;

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