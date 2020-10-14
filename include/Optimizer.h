/* #ifndef OPTIMIZER_H
#define OPTIMIZER_H
#include<unistd.h>
#include<string>
#include<thread>
#include "opencv2/opencv.hpp"


namespace VITAMINE{

    class Optimizer{
        
    public:
        Optimizer{}; //default constructor
        Optimizer{const Optimizer& rhs}; //copy constructor
        ~Optimizer{}; //destructor 
        //TODO: smart pointer

        Optimizer& operator=(const Optimizer& rhs()){};//copy assignment operator
    private:
    //cv::Mat

    };

}//namespace VITAMINE

#endif // OPTIMIZER_H
 */