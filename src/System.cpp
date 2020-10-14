#include "System.h"

namespace VITAMINE
{

    System::System(const string &strSettingsFile, const bool bUseViewer)
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
        cout<<"?"<<endl;
        //Create the Map
        mpMap = make_shared<Map>(); 

        //Create Drawers. These are used by the Viewer
        //mpMapDrawer = make_shared<MapDrawer>(mpMap, strSettingsFile); 
        //mpFrameDrawer = make_shared<FrameDrawer>(mpMap); 
        

        /* //Initialize the Tracking thread
        //(it will live in the main thread of execution, the one that called this constructor)
        mpTracker = make_shared<Tracker>(this, mpVocabulary, mpFrameDrawer, mpMapDrawer,
                                mpMap, mpKeyFrameDatabase, strSettingsFile, mSensor);

        //Initialize the Local Mapping thread and launch
        mpLocalMapper = make_shared<Mapper>(mpMap, mSensor==MONOCULAR);
        mptLocalMapping = new thread(&ORB_SLAM2::LocalMapping::Run,mpLocalMapper);

        //Initialize the Viewer thread and launch
        if(bUseViewer)
        {
            mpViewer = new Viewer(this, mpFrameDrawer,mpMapDrawer,mpTracker,strSettingsFile);
            mptViewer = new thread(&Viewer::Run, mpViewer);
            mpTracker->SetViewer(mpViewer);
        }

        //Set pointers between threads
        mpTracker->SetLocalMapper(mpLocalMapper);
        mpLocalMapper->SetTracker(mpTracker); */
    }       

    cv::Mat System::track(const cv::Mat &frame){
        //TODO: call the tracker
    }
    void System::Shutdown(){

    }
    
}