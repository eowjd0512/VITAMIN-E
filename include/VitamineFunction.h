/* #ifndef FUNCTION_H
#define FUNCTION_H
#include<unistd.h>
#include<string>
#include<thread>
#include "opencv2/opencv.hpp"


namespace VITAMINE{

    class VitamineFunction{
        
    public:
        VitamineFunction{}; //default constructor
        VitamineFunction{const VitamineFunction& rhs}; //copy constructor
        ~VitamineFunction{}; //destructor 
        //TODO: smart pointer

        VitamineFunction& operator=(const VitamineFunction& rhs()){};//copy assignment operator
    
        const void curvature();
        const void p_fn();
        const void w_fn();
        const void local_maxima();
        const void get_dominant_motion();
        const void hill_climb();
        const void vitatrack();
    private:
    //cv::Mat

    };

}//namespace VITAMINE

#endif // FUNCTION_H
 */