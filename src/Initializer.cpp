#include "Initializer.h"
#include <random>
#include<thread>
#include <opencv2/sfm/fundamental.hpp>
#include <opencv2/sfm/robust.hpp>

namespace VITAMINE
{

    Initializer::Initializer(const Frame* ReferenceFrame, float sigma, int iterations, VitamineFunction* vitamineFunction)
    :mVitamineFunction(vitamineFunction), initialFrame_idx(ReferenceFrame->mnId), mK(ReferenceFrame->mK.clone()), mSigma(sigma), mMaxIterations(iterations){

    }

    bool Initializer::Initialize(const Frame* CurrentFrame, 
                cv::Mat &R21, cv::Mat &t21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated, vector<unsigned int>& idx){
        // Fill structures with current keypoints and matches with reference frame
   
        pt1.clear();
        pt2.clear();

        for(auto i=0; i<mVitamineFunction->tf.size(); i++){
            if(mVitamineFunction->tf[i]->pt_history.count(initialFrame_idx)){
                idx.push_back(i);
                Point p1 = mVitamineFunction->tf[i]->pt_history[CurrentFrame->mnId].first;
                Point p2 = mVitamineFunction->tf[i]->pt_history[initialFrame_idx].first;
                pt1.push_back(p1);
                pt2.push_back(p2);
            }
        }


        const int N = pt1.size();
        
        // Indices for minimum set selection
        vector<size_t> vAllIndices;
        vAllIndices.reserve(N);
        vector<size_t> vAvailableIndices;

        for(int i=0; i<N; i++)
        {
            vAllIndices.push_back(i);
        }

        // Generate sets of 8 points for each RANSAC iteration
        mvSets = vector< vector<size_t> >(mMaxIterations,vector<size_t>(8,0));

        
        
        for(int it=0; it<mMaxIterations; it++)
        {   
            vAvailableIndices = vAllIndices;
            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_int_distribution<int> dis(0, vAvailableIndices.size()-1);

            // Select a minimum set
            for(size_t j=0; j<8; j++)
            {   

                int randi = dis(gen); 

                int idx = vAvailableIndices[randi];

                mvSets[it][j] = idx;

                vAvailableIndices[randi] = vAvailableIndices.back();
                vAvailableIndices.pop_back();
            }
 
        }

        vector<bool> vbMatchesInliersF;
        float SF;
        cv::Mat F;
        FindFundamental(vbMatchesInliersF,SF,F);

        return ReconstructF(vbMatchesInliersF,F,mK,R21,t21,vP3D,vbTriangulated,1.5,50);
    }

    void Initializer::FindFundamental(vector<bool> &vbMatchesInliers, float &score, cv::Mat &F21)
    {
        // Number of putative matches
        const int N = pt1.size();

        // Normalize coordinates
        vector<cv::Point2f> vPn1, vPn2;
        cv::Mat T1, T2;
        Normalize(pt1,vPn1, T1);
        Normalize(pt2,vPn2, T2);
        cv::Mat T2t = T2.t();

        // Best Results variables
        score = 0.0;
        vbMatchesInliers = vector<bool>(N,false);

        // Iteration variables
        vector<cv::Point2f> vPn1i(8);
        vector<cv::Point2f> vPn2i(8);
        cv::Mat F21i;
        vector<bool> vbCurrentInliers(N,false);
        float currentScore;

        // Perform all RANSAC iterations and save the solution with highest score
        for(int it=0; it<mMaxIterations; it++)
        {
            // Select a minimum set
            for(int j=0; j<8; j++)
            {
                int idx = mvSets[it][j];

                vPn1i[j] = vPn1[idx];
                vPn2i[j] = vPn2[idx];
            }

            cv::Mat Fn = ComputeF21(vPn1i,vPn2i);

            F21i = T2t*Fn*T1;

            currentScore = CheckFundamental(F21i, vbCurrentInliers, mSigma);

            if(currentScore>score)
            {
                F21 = F21i.clone();
                vbMatchesInliers = vbCurrentInliers;
                score = currentScore;
            }
        }
    }

    cv::Mat Initializer::ComputeF21(const vector<cv::Point2f> &vP1,const vector<cv::Point2f> &vP2)
    {
        const int N = vP1.size();

        cv::Mat A(N,9,CV_32F);

        for(int i=0; i<N; i++)
        {
            const float u1 = vP1[i].x;
            const float v1 = vP1[i].y;
            const float u2 = vP2[i].x;
            const float v2 = vP2[i].y;

            A.at<float>(i,0) = u2*u1;
            A.at<float>(i,1) = u2*v1;
            A.at<float>(i,2) = u2;
            A.at<float>(i,3) = v2*u1;
            A.at<float>(i,4) = v2*v1;
            A.at<float>(i,5) = v2;
            A.at<float>(i,6) = u1;
            A.at<float>(i,7) = v1;
            A.at<float>(i,8) = 1;
        }

        cv::Mat u,w,vt;

        cv::SVDecomp(A,w,u,vt,cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

        cv::Mat Fpre = vt.row(8).reshape(0, 3);

        cv::SVDecomp(Fpre,w,u,vt,cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

        w.at<float>(2)=0;

        return  u*cv::Mat::diag(w)*vt;
    }

    float Initializer::CheckFundamental(const cv::Mat &F21, vector<bool> &vbMatchesInliers, float sigma)
    {
        const int N = pt1.size();

        const float f11 = F21.at<float>(0,0);
        const float f12 = F21.at<float>(0,1);
        const float f13 = F21.at<float>(0,2);
        const float f21 = F21.at<float>(1,0);
        const float f22 = F21.at<float>(1,1);
        const float f23 = F21.at<float>(1,2);
        const float f31 = F21.at<float>(2,0);
        const float f32 = F21.at<float>(2,1);
        const float f33 = F21.at<float>(2,2);

        vbMatchesInliers.resize(N);

        float score = 0;

        const float th = 3.841;
        const float thScore = 5.991;

        const float invSigmaSquare = 1.0/(sigma*sigma);

        for(int i=0; i<N; i++)
        {
            bool bIn = true;

            const cv::Point &kp1 = pt1[i];
            const cv::Point &kp2 = pt2[i];
            const float u1 = kp1.x;
            const float v1 = kp1.y;
            const float u2 = kp2.x;
            const float v2 = kp2.y;

            // Reprojection error in second image
            // l2=F21x1=(a2,b2,c2)

            const float a2 = f11*u1+f12*v1+f13;
            const float b2 = f21*u1+f22*v1+f23;
            const float c2 = f31*u1+f32*v1+f33;

            const float num2 = a2*u2+b2*v2+c2;

            const float squareDist1 = num2*num2/(a2*a2+b2*b2);

            const float chiSquare1 = squareDist1*invSigmaSquare;

            if(chiSquare1>th)
                bIn = false;
            else
                score += thScore - chiSquare1;

            // Reprojection error in second image
            // l1 =x2tF21=(a1,b1,c1)

            const float a1 = f11*u2+f21*v2+f31;
            const float b1 = f12*u2+f22*v2+f32;
            const float c1 = f13*u2+f23*v2+f33;

            const float num1 = a1*u1+b1*v1+c1;

            const float squareDist2 = num1*num1/(a1*a1+b1*b1);

            const float chiSquare2 = squareDist2*invSigmaSquare;

            if(chiSquare2>th)
                bIn = false;
            else
                score += thScore - chiSquare2;

            if(bIn)
                vbMatchesInliers[i]=true;
            else
                vbMatchesInliers[i]=false;
        }

        return score;
    }

    bool Initializer::ReconstructF(vector<bool> &vbMatchesInliers, cv::Mat &F21, cv::Mat &K,
                                cv::Mat &R21, cv::Mat &t21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated, float minParallax, int minTriangulated)
    {

        int N=0;
        for(size_t i=0, iend = vbMatchesInliers.size() ; i<iend; i++)
            if(vbMatchesInliers[i])
                N++;

        // Compute Essential Matrix from Fundamental Matrix
        cv::Mat E21 = K.t()*F21*K;

        cv::Mat R1, R2, t;

        // Recover the 4 motion hypotheses
        DecomposeE(E21,R1,R2,t);  

        cv::Mat t1=t;
        cv::Mat t2=-t;

        // Reconstruct with the 4 hyphoteses and check
        vector<cv::Point3f> vP3D1, vP3D2, vP3D3, vP3D4;
        vector<bool> vbTriangulated1,vbTriangulated2,vbTriangulated3, vbTriangulated4;
        float parallax1,parallax2, parallax3, parallax4;

        int nGood1 = CheckRT(R1,t1,pt1,pt2,vbMatchesInliers,K, vP3D1, 4.0, vbTriangulated1, parallax1);
        int nGood2 = CheckRT(R2,t1,pt1,pt2,vbMatchesInliers,K, vP3D2, 4.0, vbTriangulated2, parallax2);
        int nGood3 = CheckRT(R1,t2,pt1,pt2,vbMatchesInliers,K, vP3D3, 4.0, vbTriangulated3, parallax3);
        int nGood4 = CheckRT(R2,t2,pt1,pt2,vbMatchesInliers,K, vP3D4, 4.0, vbTriangulated4, parallax4);
  
        int maxGood = max(nGood1,max(nGood2,max(nGood3,nGood4)));

        R21 = cv::Mat();
        t21 = cv::Mat();

        int nMinGood = max(static_cast<int>(0.9*N),minTriangulated);

        int nsimilar = 0;
        if(nGood1>0.7*maxGood)
            nsimilar++;
        if(nGood2>0.7*maxGood)
            nsimilar++;
        if(nGood3>0.7*maxGood)
            nsimilar++;
        if(nGood4>0.7*maxGood)
            nsimilar++;

        // If there is not a clear winner or not enough triangulated points reject initialization
        if(maxGood<nMinGood || nsimilar>1)
        {
            return false;
        }

        // If best reconstruction has enough parallax initialize
        if(maxGood==nGood1)
        {
            if(parallax1>minParallax)
            {
                vP3D = vP3D1;
                vbTriangulated = vbTriangulated1;

                R1.copyTo(R21);
                t1.copyTo(t21);
                return true;
            }
        }else if(maxGood==nGood2)
        {
            if(parallax2>minParallax)
            {
                vP3D = vP3D2;
                vbTriangulated = vbTriangulated2;

                R2.copyTo(R21);
                t1.copyTo(t21);
                return true;
            }
        }else if(maxGood==nGood3)
        {
            if(parallax3>minParallax)
            {
                vP3D = vP3D3;
                vbTriangulated = vbTriangulated3;

                R1.copyTo(R21);
                t2.copyTo(t21);
                return true;
            }
        }else if(maxGood==nGood4)
        {
            if(parallax4>minParallax)
            {
                vP3D = vP3D4;
                vbTriangulated = vbTriangulated4;

                R2.copyTo(R21);
                t2.copyTo(t21);
                return true;
            }
        }

        return false;
    }

    void Initializer::Triangulate(const cv::Point &kp1, const cv::Point &kp2, const cv::Mat &P1, const cv::Mat &P2, cv::Mat &x3D)
    {
        cv::Mat A(4,4,CV_32F);

        A.row(0) = kp1.x*P1.row(2)-P1.row(0);
        A.row(1) = kp1.y*P1.row(2)-P1.row(1);
        A.row(2) = kp2.x*P2.row(2)-P2.row(0);
        A.row(3) = kp2.y*P2.row(2)-P2.row(1);

        cv::Mat u,w,vt;
        cv::SVD::compute(A,w,u,vt,cv::SVD::MODIFY_A| cv::SVD::FULL_UV);
        x3D = vt.row(3).t();
        x3D = x3D.rowRange(0,3)/x3D.at<float>(3);
    }

    void Initializer::Normalize(const vector<cv::Point> &vKeys, vector<cv::Point2f> &vNormalizedPoints, cv::Mat &T)
    {
        float meanX = 0;
        float meanY = 0;
        const int N = vKeys.size();

        vNormalizedPoints.resize(N);

        for(int i=0; i<N; i++)
        {
            meanX += vKeys[i].x;
            meanY += vKeys[i].y;
        }

        meanX = meanX/N;
        meanY = meanY/N;

        float meanDevX = 0;
        float meanDevY = 0;

        for(int i=0; i<N; i++)
        {
            vNormalizedPoints[i].x = vKeys[i].x - meanX;
            vNormalizedPoints[i].y = vKeys[i].y - meanY;

            meanDevX += fabs(vNormalizedPoints[i].x);
            meanDevY += fabs(vNormalizedPoints[i].y);
        }

        meanDevX = meanDevX/N;
        meanDevY = meanDevY/N;

        float sX = 1.0/meanDevX;
        float sY = 1.0/meanDevY;

        for(int i=0; i<N; i++)
        {
            vNormalizedPoints[i].x = vNormalizedPoints[i].x * sX;
            vNormalizedPoints[i].y = vNormalizedPoints[i].y * sY;
        }

        T = cv::Mat::eye(3,3,CV_32F);
        T.at<float>(0,0) = sX;
        T.at<float>(1,1) = sY;
        T.at<float>(0,2) = -meanX*sX;
        T.at<float>(1,2) = -meanY*sY;
    }


    int Initializer::CheckRT(const cv::Mat &R, const cv::Mat &t, const vector<cv::Point> &vKeys1, const vector<cv::Point> &vKeys2,
                        vector<bool> &vbMatchesInliers, const cv::Mat &K, vector<cv::Point3f> &vP3D, float th2, vector<bool> &vbGood, float &parallax)
    {
        // Calibration parameters
        const float fx = K.at<float>(0,0);
        const float fy = K.at<float>(1,1);
        const float cx = K.at<float>(0,2);
        const float cy = K.at<float>(1,2);

        vbGood = vector<bool>(vKeys1.size(),false);
        vP3D.resize(vKeys1.size());

        vector<float> vCosParallax;
        vCosParallax.reserve(vKeys1.size());

        // Camera 1 Projection Matrix K[I|0]
        cv::Mat P1(3,4,CV_32F,cv::Scalar(0));
        K.copyTo(P1.rowRange(0,3).colRange(0,3));

        cv::Mat O1 = cv::Mat::zeros(3,1,CV_32F);

        // Camera 2 Projection Matrix K[R|t]
        
        Mat R_, t_;
        R.convertTo(R_, CV_32F);
        t.convertTo(t_, CV_32F);
        //R_ = -R_.t();
        //t_ = -R_.t() * t_;

        cv::Mat P2(3,4,CV_32F);
        R_.copyTo(P2.rowRange(0,3).colRange(0,3));
        t_.copyTo(P2.rowRange(0,3).col(3));
        P2 = K*P2;
      

        cv::Mat O2 = -R_.t()*t_;

        int nGood=0;

        vector<int> error(10,0);

        for(size_t i=0, iend=vKeys1.size();i<iend;i++)
        {
            if(!vbMatchesInliers[i])
                continue;

            const cv::Point &kp1 = vKeys1[i];
            const cv::Point &kp2 = vKeys2[i];
            cv::Mat p3dC1;

            Triangulate(kp1,kp2,P1,P2,p3dC1);

            if(!isfinite(p3dC1.at<float>(0)) || !isfinite(p3dC1.at<float>(1)) || !isfinite(p3dC1.at<float>(2)))
            {
                vbGood[i]=false;
                error[0]++;
                continue;
            }

            // Check parallax
            cv::Mat normal1 = p3dC1 - O1;
            float dist1 = cv::norm(normal1);

            cv::Mat normal2 = p3dC1 - O2;
            float dist2 = cv::norm(normal2);

            float cosParallax = normal1.dot(normal2)/(dist1*dist2);

            // Check depth in front of first camera (only if enough parallax, as "infinite" points can easily go to negative depth)
            if(p3dC1.at<float>(2)<=0 && cosParallax<0.99998){
                error[1]++;
                continue;
            }
                

            // Check depth in front of second camera (only if enough parallax, as "infinite" points can easily go to negative depth)
            cv::Mat p3dC2 = R_*p3dC1+t_;

            if(p3dC2.at<float>(2)<=0 && cosParallax<0.99998)
            {
                error[2]++;
                continue;
            }

            // Check reprojection error in first image
            float im1x, im1y;
            float invZ1 = 1.0/p3dC1.at<float>(2);
            im1x = fx*p3dC1.at<float>(0)*invZ1+cx;
            im1y = fy*p3dC1.at<float>(1)*invZ1+cy;

            float squareError1 = (im1x-kp1.x)*(im1x-kp1.x)+(im1y-kp1.y)*(im1y-kp1.y);

            if(squareError1>th2)
            {
                error[3]++;
                continue;
            }

            // Check reprojection error in second image
            float im2x, im2y;
            float invZ2 = 1.0/p3dC2.at<float>(2);
            im2x = fx*p3dC2.at<float>(0)*invZ2+cx;
            im2y = fy*p3dC2.at<float>(1)*invZ2+cy;

            float squareError2 = (im2x-kp2.x)*(im2x-kp2.x)+(im2y-kp2.y)*(im2y-kp2.y);

            if(squareError2>th2)
            {
                error[4]++;
                continue;
            }

            vCosParallax.push_back(cosParallax);
            vP3D[i] = cv::Point3f(p3dC1.at<float>(0),p3dC1.at<float>(1),p3dC1.at<float>(2));
            nGood++;

            if(cosParallax<0.99998)
                vbGood[i]=true;
        }

        if(nGood>0)
        {
            sort(vCosParallax.begin(),vCosParallax.end());

            size_t idx = min(50,int(vCosParallax.size()-1));
            parallax = acos(vCosParallax[idx])*180/CV_PI;
        }
        else
            parallax=0;

        cout<<"errors:"<<error[0]<<" "<<error[1]<<" "<<error[2]<<" "<<error[3]<<" "<<error[4]<<endl;
        return nGood;
    }

    void Initializer::DecomposeE(const cv::Mat &E, cv::Mat &R1, cv::Mat &R2, cv::Mat &t)
    {
        cv::Mat u,w,vt;
        cv::SVD::compute(E,w,u,vt);

        u.col(2).copyTo(t);
        t=t/cv::norm(t);

        cv::Mat W(3,3,CV_32F,cv::Scalar(0));
        W.at<float>(0,1)=-1;
        W.at<float>(1,0)=1;
        W.at<float>(2,2)=1;

        R1 = u*W*vt;
        if(cv::determinant(R1)<0)
            R1=-R1;

        R2 = u*W.t()*vt;
        if(cv::determinant(R2)<0)
            R2=-R2;
    }
}//namespace VITAMINE