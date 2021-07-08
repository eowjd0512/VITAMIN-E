#include "System.h"

namespace VITAMINE
{

    System::System(const string &strSettingsFile, const bool bUseViewer)
    :mpViewer(static_cast<Viewer*>(NULL)), mbReset(false)
    {
        // Output welcome message
        /* cout << endl <<
        "ORB-SLAM2 Copyright (C) 2014-2016 Raul Mur-Artal, University of Zaragoza." << endl <<
        "This program comes with ABSOLUTELY NO WARRANTY;" << endl  <<
        "This is free software, and you are welcome to redistribute it" << endl <<
        "under certain conditions. See LICENSE.txt." << endl << endl; */

        //Check settings file
        cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
        if(!fsSettings.isOpened())
        {
        cerr << "Failed to open settings file at: " << strSettingsFile << endl;
        exit(-1);
        }

        //Create the Map
        mpMap = new Map();

        //Create Drawers. These are used by the Viewer
        mpMapDrawer = new MapDrawer(mpMap, strSettingsFile); 
        mpFrameDrawer = new FrameDrawer(mpMap); 
        

        //Initialize the Tracking thread
        //(it will live in the main thread of execution, the one that called this constructor)
        mpTracker = new Tracker(this, mpFrameDrawer, mpMapDrawer,
                                mpMap, strSettingsFile);

        //Initialize the Local Mapping thread and launch
        mpMapper = new Mapper(mpMap);
        mptMapping = new thread(&Mapper::Run,mpMapper);

        //Initialize the Viewer thread and launch
        if(bUseViewer)
        {
            mpViewer = new Viewer(this, mpFrameDrawer,mpMapDrawer,mpTracker,strSettingsFile);
            mptViewer = new thread(&Viewer::Run, mpViewer);
            mpTracker->SetViewer(mpViewer);
        }

        //Set pointers between threads
        mpTracker->SetMapper(mpMapper);
        mpMapper->SetTracker(mpTracker);

    }       

    cv::Mat System::Track(const cv::Mat &im){
        //TODO: call the tracker

        // Check reset
        {
            unique_lock<mutex> lock(mMutexReset);
            if(mbReset)
            {
                mpTracker->Reset();
                mbReset = false;
            }
        }

        cv::Mat Tcw = mpTracker->GrabImageMonocular(im);

        unique_lock<mutex> lock2(mMutexState);
        mTrackingState = mpTracker->mState;
        mTrackedMapPoints = mpTracker->mCurrentFrame->mvpMapPoints;
        mTrackedKeyPointsUn = mpTracker->mCurrentFrame->mvKeysUn;

        return Tcw;

    }
    void System::Reset()
    {
        unique_lock<mutex> lock(mMutexReset);
        mbReset = true;
    }

    void System::Shutdown(){

    }
    
}