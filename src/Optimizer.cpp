#include "Optimizer.h"
#include "Converter.h"
#include "g2o_Imitiator.h"
#include "thirdParty/g2o/g2o/core/block_solver.h"
#include "thirdParty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "thirdParty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "thirdParty/g2o/g2o/types/types_six_dof_expmap.h"
#include "thirdParty/g2o/g2o/core/robust_kernel_impl.h"
#include "thirdParty/g2o/g2o/solvers/linear_solver_dense.h"
#include "thirdParty/g2o/g2o/types/types_seven_dof_expmap.h"

#include<Eigen/StdVector>

namespace VITAMINE
{

/* void Optimizer::GlobalBundleAdjustemnt(Map* pMap, int nIterations, bool* pbStopFlag, const unsigned long nLoopKF, const bool bRobust)
{
    vector<Frame*> vpFs = pMap->GetAllFrames();
    vector<MapPoint*> vpMP = pMap->GetAllMapPoints();
    BundleAdjustment(vpFs,vpMP,nIterations,pbStopFlag, nLoopKF, bRobust);
}


void Optimizer::BundleAdjustment(const vector<Frame*> &vpFs, const vector<MapPoint *> &vpMP,
                                 int nIterations, bool* pbStopFlag, const unsigned long nLoopKF, const bool bRobust)
{
    vector<bool> vbNotIncludedMP;
    vbNotIncludedMP.resize(vpMP.size());

    SparseOptimizer optimizer;

    long unsigned int maxKFid = 0;

    // Set KeyFrame vertices
    for(size_t i=0; i<vpFs.size(); i++)
    {
        Frame* pF = vpFs[i];
        cout<<i<<"th pKF pose: "<< pF->GetPose()<<", id: "<<pF->mnId<<endl;
        if(pF->isBad())
            continue;
        VertexSE3Expmap * vSE3 = new VertexSE3Expmap();
        vSE3->setEstimate(Converter::toSE3Quat(pF->GetPose()));
        vSE3->setId(pF->mnId);
        vSE3->setFixed(pF->mnId==0);
        optimizer.addVertex(vSE3);
        if(pF->mnId>maxKFid)
            maxKFid=pF->mnId;
    }

    const float thHuber2D = sqrt(5.99);
    const float thHuber3D = sqrt(7.815);

    // Set MapPoint vertices
    for(size_t i=0; i<vpMP.size(); i++)
    {
        MapPoint* pMP = vpMP[i];
        if(pMP->isBad())
            continue;
        VertexSBAPointXYZ* vPoint = new VertexSBAPointXYZ();
        vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
        const int id = pMP->mnId+maxKFid+1;
        vPoint->setId(id);
        //vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);

       const map<Frame*,size_t> observations = pMP->GetObservations();

        int nEdges = 0;
        //SET EDGES
        for(map<Frame*,size_t>::const_iterator mit=observations.begin(); mit!=observations.end(); mit++)
        {

            Frame* pF = mit->first;
            if(pF->isBad() || pF->mnId>maxKFid)
                continue;

            nEdges++;

            const cv::Point &pt = pF->mvTrackedKeys[mit->second]; //TODO:

            Eigen::Matrix<double,2,1> obs;
            obs << pt.x, pt.y; //TODO: get projected points

            EdgeSE3ProjectXYZ* e = new EdgeSE3ProjectXYZ();

            e->setVertexPoint(optimizer.vertex_point(id));
            e->setVertexPose(optimizer.vertex_pose(pF->mnId));
            e->setMeasurement(obs);
            //const float &invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
            e->setInformation(Eigen::Matrix2d::Identity());//*invSigma2);

            if(bRobust)
            {
                RobustKernelHuber* rk = new RobustKernelHuber();
                e->setRobustKernel(rk);
                rk->setDelta(thHuber2D);
            }

            e->fx = pF->fx;
            e->fy = pF->fy;
            e->cx = pF->cx;
            e->cy = pF->cy;

            optimizer.addEdge(e);
            
        }

        if(nEdges==0)
        {
            optimizer.removeVertex(vPoint);
            vbNotIncludedMP[i]=true;
        }
        else
        {
            vbNotIncludedMP[i]=false;
        }
    }

    // Optimize!
    //optimizer.initializeOptimization();
    optimizer.optimize(nIterations);

    // Recover optimized data

    //Keyframes
    for(size_t i=0; i<vpFs.size(); i++)
    {
        Frame* pF = vpFs[i];
        if(pF->isBad())
            continue;
        VertexSE3Expmap* vSE3 = optimizer.vertex_pose(pF->mnId);
        SE3Quat SE3quat = vSE3->estimate();
        if(nLoopKF==0)
        {
            pF->SetPose(Converter::toCvMat(SE3quat));
        }

    }

    //Points
    for(size_t i=0; i<vpMP.size(); i++)
    {
        if(vbNotIncludedMP[i])
            continue;

        MapPoint* pMP = vpMP[i];

        if(pMP->isBad())
            continue;
        VertexSBAPointXYZ* vPoint = optimizer.vertex_point(pMP->mnId+maxKFid+1);

        if(nLoopKF==0)
        {
            pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
            //pMP->UpdateNormalAndDepth(); //TODO:
        }
    }
} */

void Optimizer::G2O_GlobalBundleAdjustemnt(Map* pMap, int nIterations, bool* pbStopFlag, const unsigned long nLoopKF, const bool bRobust)
{
    vector<Frame*> vpFs = pMap->GetAllFrames();
    vector<MapPoint*> vpMP = pMap->GetAllMapPoints();
    G2O_BundleAdjustment(vpFs,vpMP,nIterations,pbStopFlag, nLoopKF, bRobust);
}


void Optimizer::G2O_BundleAdjustment(const vector<Frame*> &vpFs, const vector<MapPoint *> &vpMP,
                                 int nIterations, bool* pbStopFlag, const unsigned long nLoopKF, const bool bRobust)
{
    vector<bool> vbNotIncludedMP;
    vbNotIncludedMP.resize(vpMP.size());

    g2o::SparseOptimizer optimizer;
    g2o::BlockSolver_6_3::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

    g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    if(pbStopFlag)
        optimizer.setForceStopFlag(pbStopFlag);

    long unsigned int maxKFid = 0;

    // Set KeyFrame vertices
    for(size_t i=0; i<vpFs.size(); i++)
    {
        Frame* pF = vpFs[i];
        cout<<i<<"th pKF pose: "<< pF->GetPose()<<", id: "<<pF->mnId<<endl;
        if(pF->isBad())
            continue;
        g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(Converter::toSE3Quat(pF->GetPose())); //TODO:
        vSE3->setId(pF->mnId);
        vSE3->setFixed(pF->mnId==0);
        optimizer.addVertex(vSE3);
        if(pF->mnId>maxKFid)
            maxKFid=pF->mnId;
    }
    const float thHuber2D = sqrt(5.99);
    const float thHuber3D = sqrt(7.815);

    // Set MapPoint vertices
    for(size_t i=0; i<vpMP.size(); i++)
    {
        MapPoint* pMP = vpMP[i];
        if(pMP->isBad())
            continue;
        g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
        vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos())); //TODO:
        const int id = pMP->mnId+maxKFid+1;
        vPoint->setId(id);
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);

       const map<Frame*,size_t> observations = pMP->GetObservations();

        int nEdges = 0;
        //SET EDGES
        for(map<Frame*,size_t>::const_iterator mit=observations.begin(); mit!=observations.end(); mit++)
        {

            Frame* pF = mit->first;
            if(pF->isBad() || pF->mnId>maxKFid)
                continue;

            nEdges++;

            const cv::Point &pt = pF->mvTrackedKeys[mit->second]; //TODO:

            Eigen::Matrix<double,2,1> obs;
            obs << pt.x, pt.y; //TODO: get projected points

            g2o::EdgeSE3ProjectXYZ* e = new g2o::EdgeSE3ProjectXYZ();

            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pF->mnId)));
            e->setMeasurement(obs);
            //const float &invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
            e->setInformation(Eigen::Matrix2d::Identity());//*invSigma2);

            if(bRobust)
            {
                g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber();
                e->setRobustKernel(rk);
                rk->setDelta(thHuber2D);
            }

            e->fx = pF->fx;
            e->fy = pF->fy;
            e->cx = pF->cx;
            e->cy = pF->cy;

            optimizer.addEdge(e);
            
        }
        if(nEdges==0)
        {
            optimizer.removeVertex(vPoint);
            vbNotIncludedMP[i]=true;
        }
        else
        {
            vbNotIncludedMP[i]=false;
        }
    }

    // Optimize!
    optimizer.initializeOptimization();
    optimizer.optimize(nIterations);

    // Recover optimized data

    //Keyframes
    for(size_t i=0; i<vpFs.size(); i++)
    {
        Frame* pF = vpFs[i];
        if(pF->isBad())
            continue;
        g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pF->mnId));
        g2o::SE3Quat SE3quat = vSE3->estimate();
        if(nLoopKF==0)
        {
            pF->SetPose(Converter::toCvMat(SE3quat));
        }
        /* else
        {
            pF.mTcwGBA.create(4,4,CV_32F);
            Converter::toCvMat(SE3quat).copyTo(pKF->mTcwGBA);
            pKF->mnBAGlobalForKF = nLoopKF;
        } */
    }

    //Points
    for(size_t i=0; i<vpMP.size(); i++)
    {
        if(vbNotIncludedMP[i])
            continue;

        MapPoint* pMP = vpMP[i];

        if(pMP->isBad())
            continue;
        g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(pMP->mnId+maxKFid+1));

        if(nLoopKF==0)
        {
            pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
            //pMP->UpdateNormalAndDepth(); //TODO:
        }
        /* else
        {
            pMP->mPosGBA.create(3,1,CV_32F);
            Converter::toCvMat(vPoint->estimate()).copyTo(pMP->mPosGBA);
            pMP->mnBAGlobalForKF = nLoopKF;
        } */
    }

    }
}//namespace VITAMINE