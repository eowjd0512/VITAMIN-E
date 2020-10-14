/* #ifndef VIEWER_H
#define VIEWER_H
#include<unistd.h>
#include<string>
#include<thread>
#include "opencv2/opencv.hpp"


namespace VITAMINE{

    class Viewer{
        
    public:
        Viewer{}; //default constructor
        Viewer{const Viewer& rhs}; //copy constructor
        ~Viewer{}; //destructor 
        //TODO: smart pointer

        Viewer& operator=(const Viewer& rhs()){};//copy assignment operator
    private:
    //cv::Mat

    };

}//namespace VITAMINE

#endif // VIEWER_H
 */