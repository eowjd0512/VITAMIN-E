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

namespace VITAMINE{

class Optimizer{
    
public:
    Optimizer(){}; //default constructor
    Optimizer(const Optimizer& rhs){}; //copy constructor
    ~Optimizer(){}; //destructor 
    //TODO: smart pointer

    void static BundleAdjustment(const std::vector<Frame> &vpF, const std::vector<MapPoint*> &vpMP,
                                int nIterations = 5, bool *pbStopFlag=NULL, const unsigned long nLoopKF=0,
                                const bool bRobust = true);
    void static GlobalBundleAdjustemnt(Map* pMap, int nIterations=5, bool *pbStopFlag=NULL,
                                       const unsigned long nLoopKF=0, const bool bRobust = true);
    
};
class SparseOptimizer{
public:
    SparseOptimizer();
    ~SparseOptimizer();

    VertexSE3Expmap* vertex_pose(unsigned long mnId);
    VertexSBAPointXYZ* vertex_point(unsigned long mnId);

    void addVertex(VertexSE3Expmap* VertexSE3Expmap);//for pose
    void addVertex(VertexSBAPointXYZ* VertexSBAPointXYZ);//for point
    void addEdge(EdgeSE3ProjectXYZ* EdgeSE3ProjectXYZ);
    void removeVertex(VertexSBAPointXYZ* VertexSBAPointXYZ);
    void optimize(int nIterations);

};

class VertexSE3Expmap{
public:
    VertexSE3Expmap();
    ~VertexSE3Expmap();

    void setEstimate(const SE3Quat& SE3Quat);
    void setId(unsigned long mnId);
    void setFixed(bool flag);
    SE3Quat estimate();
};

class VertexSBAPointXYZ{
public:
    VertexSBAPointXYZ();
    ~VertexSBAPointXYZ();

    void setEstimate(const Eigen::Vector3d& vec);
    void setId(unsigned long mnId);
    Eigen::Vector3d estimate();

};

class EdgeSE3ProjectXYZ{
public:
    EdgeSE3ProjectXYZ();
    ~EdgeSE3ProjectXYZ();

    void setVertexPoint(VertexSBAPointXYZ* VertexSBAPointXYZ);
    void setVertexPose(VertexSE3Expmap* VertexSE3Expmap);
    void setMeasurement(const Eigen::Matrix<double,2,1>& obs);
    void setInformation(const Eigen::Matrix2d& info);
    void setRobustKernel(RobustKernelHuber* RobustKernelHuber);

float fx;
float fy;
float cx;
float cy;

};

class RobustKernelHuber{
public:
    RobustKernelHuber();
    ~RobustKernelHuber();
    void setDelta(const float thHuber2D);
};

class SE3Quat{

public:
SE3Quat(const Eigen::Matrix<double,3,3>& R, const Eigen::Matrix<double,3,1>& t);
~SE3Quat();

Eigen::Matrix<double,4,4> to_homogeneous_matrix();
};

class Sim3{

public:
    Sim3();
    ~Sim3();
    inline Eigen::Quaterniond& rotation();
    inline Eigen::Vector3d& translation();
    inline double& scale();

};

}//namespace VITAMINE

#endif // OPTIMIZER_H
