/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef OPTIMIZER_H
#define OPTIMIZER_H
#include<unistd.h>
#include<string>
#include<thread>
#include "opencv2/opencv.hpp"
#include "Map.h"
#include<Eigen/Dense>
namespace VITAMINE{

class Optimizer{
    
public:
    Optimizer(){}; //default constructor
    Optimizer(const Optimizer& rhs){}; //copy constructor
    ~Optimizer(){}; //destructor 
    //TODO: smart pointer

    /* void static BundleAdjustment(const std::vector<Frame*> &vpF, const std::vector<MapPoint*> &vpMP,
                                int nIterations = 5, bool *pbStopFlag=NULL, const unsigned long nLoopKF=0,
                                const bool bRobust = true);
    void static GlobalBundleAdjustemnt(Map* pMap, int nIterations=5, bool *pbStopFlag=NULL,
                                       const unsigned long nLoopKF=0, const bool bRobust = true);
     */
    void G2O_BundleAdjustment(const std::vector<Frame*> &vpF, const std::vector<MapPoint*> &vpMP,
                                int nIterations = 5, bool *pbStopFlag=NULL, const unsigned long nLoopKF=0,
                                const bool bRobust = true);
    void G2O_GlobalBundleAdjustemnt(Map* pMap, int nIterations=5, bool *pbStopFlag=NULL,
                                       const unsigned long nLoopKF=0, const bool bRobust = true);
    

};
}//namespace VITAMINE

#endif // OPTIMIZER_H
