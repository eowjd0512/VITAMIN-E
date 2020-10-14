/* #ifndef MAPPER_H
#define MAPPER_H
#include<unistd.h>
#include<string>
#include<thread>
#include "opencv2/opencv.hpp"


namespace VITAMINE{

    class Mapper{
        
    public:
        Mapper{}; //default constructor
        Mapper{const Mapper& rhs}; //copy constructor
        ~Mapper{}; //destructor 
        //TODO: smart pointer

        Mapper& operator=(const Mapper& rhs()){};//copy assignment operator
    private:
    //cv::Mat

    };

}//namespace VITAMINE

#endif // MAPPER_H
 */