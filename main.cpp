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
     
    Mat frame; 
    while(true)  
    {  
        
        cap >> frame;  
        vitamine.track(frame);

    }  
    
    vitamine.Shutdown();
  
    return 0;  
}  