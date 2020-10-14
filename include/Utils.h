/* #ifndef UTILS_H
#define UTILS_H
#include<unistd.h>
#include<string>
#include<thread>
#include "opencv2/opencv.hpp"


namespace VITAMINE{

    class Utils{
        
    public:
        Utils{}; //default constructor
        Utils{const Utils& rhs}; //copy constructor
        ~Utils{}; //destructor 
        //TODO: smart pointer

        Utils& operator=(const Utils& rhs()){};//copy assignment operator

        const void normalize();



    private:
    //cv::Mat

    };

}//namespace VITAMINE

#endif // UTILS_H
 */