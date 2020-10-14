#include "opencv2/opencv.hpp"
#include <iostream>  
#include "System.h"

using namespace cv;  
using namespace std;  

  
int main(int argc, char **argv)  
{  

    VideoCapture cap(0);  
    if (!cap.isOpened())  
    {  
        cerr<<"cam open error"<<endl;  
    }  
  
    bool visualizer = true;
    VITAMINE::System vitamine(argv[1], visualizer);
     cout<<"6"<<endl;
    Mat frame; 
    while(true)  
    {  
        cout<<"7"<<endl;
        cap >> frame; 
        cout<<"8"<<endl; 
        vitamine.track(frame);
        cout<<"9"<<endl;
    }  
    
    vitamine.Shutdown();
  
    return 0;  
}  