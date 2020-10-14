/* #ifndef MATCHER_H
#define MATCHER_H
#include<unistd.h>
#include<string>
#include<thread>
#include "opencv2/opencv.hpp"


namespace VITAMINE{

    class Matcher{
        
    public:
        Matcher{}; //default constructor
        Matcher{const Matcher& rhs}; //copy constructor
        ~Matcher{}; //destructor 
        //TODO: smart pointer

        Matcher& operator=(const Matcher& rhs()){};//copy assignment operator

        const void to_kpt();
        const void match();
    private:
    //cv::Mat
        
    };

}//namespace VITAMINE

#endif // MATCHER_H
 */