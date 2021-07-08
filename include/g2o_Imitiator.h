/* 
#ifndef G2OIMITIATOR_H
#define G2OIMITIATOR_H
#include<unistd.h>
#include<string>
#include<thread>
#include "opencv2/opencv.hpp"
#include "Map.h"
#include <Eigen/Dense>

namespace VITAMINE{

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

#endif // G2OIMITIATOR_H
 */