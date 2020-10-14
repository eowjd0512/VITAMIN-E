/* #ifndef TRACKER_H
#define TRACKER_H
#include<unistd.h>
#include<string>
#include<thread>
#include "opencv2/opencv.hpp"


namespace VITAMINE{

    class Tracker{
        
    public:
        Tracker{}; //default constructor
        Tracker{const Tracker& rhs}; //copy constructor
        ~Tracker{}; //destructor 
        //TODO: smart pointer

        Tracker& operator=(const Tracker& rhs()){};//copy assignment operator

        const void tracking();

    private:
    //cv::Mat

    };

}//namespace VITAMINE

#endif // TRACKER_H
 */