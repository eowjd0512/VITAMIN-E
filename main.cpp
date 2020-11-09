#include "opencv2/opencv.hpp"
#include <iostream>  
#include "System.h"

using namespace cv;  
using namespace std;  

  
int main(int argc, char **argv)  
{  

    VideoCapture cap(2);  
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