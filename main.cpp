#define tum

#include "opencv2/opencv.hpp"
#include <iostream>  
#include "System.h"
#include<fstream>

using namespace cv;  
using namespace std;  

#ifdef tum
void LoadImages(const string &strFile, vector<string> &vstrImageFilenames,
                vector<double> &vTimestamps);

int main(int argc, char **argv)  
{  
    // Retrieve paths to images
    vector<string> vstrImageFilenames;
    vector<double> vTimestamps;
    string strFile = string(argv[2])+"/rgb.txt";
    cout<<strFile<<endl;
    LoadImages(strFile, vstrImageFilenames, vTimestamps);

    int nImages = vstrImageFilenames.size();

    cout<<vstrImageFilenames.size()<<endl;
  
    bool visualizer = true;
    VITAMINE::System vitamine(argv[1], visualizer);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop
    cv::Mat im;
    for(int ni=0; ni<nImages; ni++)
    {
        // Read image from file
        im = cv::imread(string(argv[2])+"/"+vstrImageFilenames[ni]);
        cout<<im.size()<<endl;
        
        double tframe = vTimestamps[ni];
        
        if(im.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(argv[2]) << "/" << vstrImageFilenames[ni] << endl;
            return 1;
        }

        // Pass the image to the SLAM system
        cout<<"?"<<endl;
        vitamine.Track(im);

        /* vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6); */
    }

    // Stop all threads
    vitamine.Shutdown();

    /* // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl; */


    vitamine.Shutdown();
  
    return 0;  
}  

void LoadImages(const string &strFile, vector<string> &vstrImageFilenames, vector<double> &vTimestamps)
{
    ifstream f;
    f.open(strFile.c_str());

    // skip first three lines
    string s0;
    getline(f,s0);
    getline(f,s0);
    getline(f,s0);

    while(!f.eof())
    {
        string s;
        getline(f,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            string sRGB;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenames.push_back(sRGB);
        }
    }
}
#endif

#ifdef webcam
int main(int argc, char **argv)  
{  

    VideoCapture cap(0);  
    if (!cap.isOpened())  
    {  
        cerr<<"cam open error"<<endl;  
    }  
  
    bool visualizer = true;
    VITAMINE::System vitamine(argv[1], visualizer);

    Mat frame; 
    int i=0;
    while(true)  
    {  

        cap >> frame;  
        resize(frame, frame, Size(640,480));
        //imshow("img", frame);
        //std::string filename = "calib_img1"+to_string(i++)+".jpg";
        //imwrite(filename, frame);
        //waitKey(0);
        vitamine.Track(frame);

    }  
    
    vitamine.Shutdown();
  
    return 0;  
}  

#endif