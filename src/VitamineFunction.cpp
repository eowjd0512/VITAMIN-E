#include "VitamineFunction.h"
#include<Eigen/Dense>
#include <omp.h>

namespace VITAMINE{

    size_t VitamineFunction::getPointIdx(unsigned long FrameId, size_t featureIdx){
        return tf[featureIdx]->pt_history[FrameId].second;
    }
    void VitamineFunction::AddMapPoint(MapPoint* pMP,const unsigned int pt_idx) noexcept{
        tf[pt_idx]->mapPoint = pMP;
    }


    int VitamineFunction::getTrackedFeatureNum(unsigned int frame_id) const noexcept{
        int cnt = 0;
        for(auto i=0; i<tf.size(); i++){
            if(tf[i]->pt_history.count(frame_id))
                cnt++;
        }
        return cnt;
    }

    const void VitamineFunction::setInitialFeatures(){
        std::vector<KeyPoint>& KeyPoints = mPrevFrame->mvKeysUn;
        
        for(int i=0; i< KeyPoints.size(); i++){

            mPrevFrame->insertTrackingFeature(KeyPoints[i].pt); //TODO: mutex?
            size_t ptIdx = mPrevFrame->TrackingFeatureSize()-1;

            std::pair<cv::Point, size_t> pt = std::make_pair(KeyPoints[i].pt, ptIdx);
            Feature* tf_ = new Feature(pt, mPrevFrame->mnId);
            
            
            tf.push_back(tf_);
        }

        size_t N = mPrevFrame->TrackingFeatureSize();
        mPrevFrame->mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));
        //mPrevFrame->mvbOutlier = vector<bool>(N,false);

    }
    const void VitamineFunction::addResidualFeatures(int cntThres){
        
        //1. get all indices of a grid that include local features
        Eigen::MatrixXi cntLocalFeatureInFrameGrid = Eigen::MatrixXi::Zero(FRAME_GRID_COLS, FRAME_GRID_ROWS);
        
        for(int i=0; i<FRAME_GRID_COLS; i++){
            for(int j=0; j<FRAME_GRID_ROWS; j++){
                if(mCurrentFrame->mGrid[i][j].size()>0){
                    cntLocalFeatureInFrameGrid(i, j) = mCurrentFrame->mGrid[i][j].size();
                }
            }
        }
        //2. tracked feature assignment to a grid
        Eigen::MatrixXi cntTrackFeatureInFrameGrid = Eigen::MatrixXi::Zero(FRAME_GRID_COLS, FRAME_GRID_ROWS);
        
        for(int i=0; i<tf.size(); i++){
            int x, y;
            if(mCurrentFrame->PosInGrid(tf[i]->pt, x, y)){
                cntTrackFeatureInFrameGrid(x, y)++;
            }
        }

        //3. check if a local feature in a grid cell already exists
        Eigen::MatrixXi diffCntFeature = cntLocalFeatureInFrameGrid - cntTrackFeatureInFrameGrid;
        std::vector<KeyPoint>& KeyPoints = mCurrentFrame->mvKeysUn;

        for(int i=0; i<FRAME_GRID_COLS; i++){
            for(int j=0; j<FRAME_GRID_ROWS; j++){
                if(diffCntFeature(i, j) >= cntThres){
                    vector<size_t>& localFeatIdx = mCurrentFrame->mGrid[i][j];
                    for(int k=0; k<localFeatIdx.size(); k++){
                        //cout<<KeyPoints[localFeatIdx[k]].pt<<endl;
                        mCurrentFrame->insertTrackingFeature(KeyPoints[localFeatIdx[k]].pt);
                        size_t ptIdx = mCurrentFrame->TrackingFeatureSize()-1;

                        std::pair<cv::Point, size_t> pt = std::make_pair(KeyPoints[localFeatIdx[k]].pt, ptIdx);
                        
                        Feature* tf_ = new Feature(pt, mCurrentFrame->mnId);
                        tf.push_back(tf_);

                        mCurrentFrame->mvpMapPoints.push_back(static_cast<MapPoint*>(NULL));
                    }
                }
            }
        }


    }
    const void VitamineFunction::drawTrackingFeatures(){
        Mat src = mCurrentFrame->img;
        cvtColor(src,src,COLOR_GRAY2BGR);
        
        RNG rng(12345);
        
        for (int i = 0; i < tf.size(); i++) {
            if(tf[i]->color[0] < 0)
                tf[i]->color = Scalar(rng.uniform(0,255), rng.uniform(0, 255), rng.uniform(0, 255));
            circle(src, tf[i]->pt, 2, tf[i]->color, -1, 8, 0);
        }
        imshow("src",src);
        waitKey(30);
    }
    
    const void VitamineFunction::loadConsecutiveFrames(Frame*prevFrame, Frame*currentFrame){
        mPrevFrame = prevFrame;
        mCurrentFrame = currentFrame;
    }
    const void VitamineFunction::getDominantMotion(const std::vector<DMatch>& good_matches){
        int match_size = good_matches.size();
        cout<<"match_size: "<<match_size<<endl;
        
        vector<Point> pt0(match_size), pt1(match_size);
        
        /* std::transform(good_matches.begin(), good_matches.end(), pt0.begin(), 
        [mPrevFrame.mvKeysUn](DMatch matches) {return mPrevFrame.mvKeysUn[matches.queryIdx];});
        std::transform(good_matches.begin(), good_matches.end(), pt1.begin(), 
        [mCurrentFrame.mvKeysUn](DMatch matches) {return mCurrentFrame.mvKeysUn[matches.trainIdx];}); */
        vector<KeyPoint>& mPrevKP = mPrevFrame->ORBKeys;
        vector<KeyPoint>& mCurrentKP = mCurrentFrame->ORBKeys;

        for(int i=0; i<match_size; i++){
            pt0[i] = mPrevKP[good_matches[i].queryIdx].pt;
            pt1[i] = mCurrentKP[good_matches[i].trainIdx].pt;
        }

        Mat mpt0(pt0);
        Mat mpt1(pt1);

        if(pt0.size()>50 && pt1.size()>50)
            Ab = estimateAffine2D(mpt0, mpt1);
        
    }
    const void VitamineFunction::vitaTrack(){
        
        if (tf.size()==0){
            setInitialFeatures();
        }
 
        vector<Point2d> predictedP_precision;
        vector<Point> predictedP;
        //vector<int> predictedP2KeyPoint_idx;

        for(int i=0; i<tf.size(); i++){
            Mat pt(tf[i]->pt);
            pt.convertTo(pt, CV_64F);
            
            Mat pred_pt_ = Ab.colRange(0,2)*pt+Ab.col(2);
            Point2d pred_pt(pred_pt_);

            if(pred_pt.x < mPrevFrame->mnMinX || pred_pt.x > mPrevFrame->mnMaxX || 
              pred_pt.y < mPrevFrame->mnMinY || pred_pt.y > mPrevFrame->mnMaxY){
                tf[i] = static_cast<Feature*>(NULL);
                tf.erase(tf.begin()+i);
                i--;
                continue;
            }

            predictedP_precision.push_back(pred_pt);
            predictedP.push_back(Point(pred_pt));
            //predictedP2KeyPoint_idx.push_back(i);
        }

        double lmd=5;
        
        std::chrono::system_clock::time_point start = std::chrono::system_clock::now();

        hill_climb(mCurrentFrame->kappa, predictedP, predictedP_precision, lmd);

        std::chrono::duration<double> sec = std::chrono::system_clock::now() - start;
        std::cout << "hill_climb time : " << sec.count() << " seconds" << std::endl;
        
        //TODO: culling repeated points using multi-dimensional map
        //

        //update pt
        assert(predictedP.size() == tf.size());
        for(int i=0; i<tf.size(); i++){
            tf[i]->pt = predictedP[i];
            tf[i]->viewIdx.push_back(mCurrentFrame->mnId);

            mCurrentFrame->insertTrackingFeature(predictedP[i]);
            size_t ptIdx = mCurrentFrame->TrackingFeatureSize()-1;

            std::pair<cv::Point, size_t> pt = std::make_pair(predictedP[i], ptIdx);
            tf[i]->pt_history[mCurrentFrame->mnId] = pt;


            //data association
            if(!tf[i]->mapPoint)
                mCurrentFrame->mvpMapPoints.push_back(static_cast<MapPoint*>(NULL));
            else{
                mCurrentFrame->mvpMapPoints.push_back(tf[i]->mapPoint); 
                tf[i]->mapPoint->AddObservation(mCurrentFrame, ptIdx);
            }
        }
      
    }
    void VitamineFunction::hill_climb_bk(const vector<Mat>& kappa, vector<Point>& pt1, vector<Point2d> pt1_, double lmd){
        //kappa_pad = np.pad(kappa, ((1,1),(1,1)),
        //    mode='constant', constant_values=-np.inf)
        size_t N = pt1.size();
        vector<bool> msk(N, true);
        
        vector<double> F(N);
        
        for(int i=0; i<N; i++){
            F[i] = static_cast<double>(kappa[0].at<uchar>(pt1[i].y, pt1[i].x));
        }   
        
        Mat kappa_pad;

        copyMakeBorder(kappa[0],kappa_pad,1,1,1,1,BORDER_DEFAULT,Scalar(0));
      
        while (true){
            
            vector<vector<double>> Fs;
            vector<Point> ds;
            
            vector<int> patch{-1, 0, 1};

            for(int i = 0; i<patch.size(); i++){
                int di = patch[i];
                for(int j = 0; j<patch.size(); j++){
                    int dj = patch[j];
                    ds.push_back(Point(di,dj));
                    if(di==0 && dj==0){
                        Fs.push_back(F);
                        continue;
                    }
                    //vector<double> d_pt(pt1.size());
                    vector<double> f(N);
                    //#pragma omp parallel for
                        for(int k=0; k<pt1.size(); k++){
                            if(msk[k]){
                                Point2d pt = Point2d(Point(pt1[k])) + Point2d(di,dj) - pt1_[k];
                                double d_pt = sqrt(pow(pt.x, 2)+pow(pt.y, 2));
                                /* int sum_f = (int)kappa_pad.at<uchar>(pt1[k].y+(1+dj-1), pt1[k].x+(1+di-1))+
                                            (int)kappa_pad.at<uchar>(pt1[k].y+(1+dj-1), pt1[k].x+(1+di+0))+
                                            (int)kappa_pad.at<uchar>(pt1[k].y+(1+dj-1), pt1[k].x+(1+di+1))+
                                            (int)kappa_pad.at<uchar>(pt1[k].y+(1+dj+0), pt1[k].x+(1+di-1))+
                                            (int)kappa_pad.at<uchar>(pt1[k].y+(1+dj+0), pt1[k].x+(1+di+0))+
                                            (int)kappa_pad.at<uchar>(pt1[k].y+(1+dj+0), pt1[k].x+(1+di+1))+
                                            (int)kappa_pad.at<uchar>(pt1[k].y+(1+dj+1), pt1[k].x+(1+di-1))+
                                            (int)kappa_pad.at<uchar>(pt1[k].y+(1+dj+1), pt1[k].x+(1+di+0))+
                                            (int)kappa_pad.at<uchar>(pt1[k].y+(1+dj+1), pt1[k].x+(1+di+1));
                                
                                double f_ = (double)sum_f/(double)6 + lmd * w_fn(d_pt, 0.1); */
                                
                                double f_ = static_cast<double>(kappa_pad.at<uchar>(pt1[k].y+(1+dj), pt1[k].x+(1+di))) + lmd * w_fn(d_pt, 0.1);
                                f[k] = f_;
                            }
                        }
                    
                    Fs.push_back(f);
        
                }
            }
            //colwise argmax
            vector<unsigned int> sel(N);
            vector<double> FsMax(N, -100000);
            for(int i=0; i<Fs.size(); i++){
                assert(N == Fs[i].size());

                for(int j=0; j<Fs[i].size(); j++){
                    if(msk[j]){
                        if(FsMax[j] < Fs[i][j]){
                            FsMax[j] = Fs[i][j];
                            sel[j] = i;
                        }
                    }
                }
            }
            F = FsMax;
            
            //recalc msk
            for(int i=0; i< N; i++){
                if(msk[i]){
                    if(sel[i]==4)
                        msk[i] = false;
                }
            }

            //update pt
            int numRest=0;
            for(int i=0; i< N; i++){
                if(msk[i]){
                    pt1[i] = pt1[i] + ds[sel[i]];
                    numRest++;
                }
            }
            if (!numRest) break;
        }


        /* for(int i=0;i<N;i++){
            cout<<Point(pt1_[i])<<" -> "<<pt1[i]<<endl;
        } */
    }

    void VitamineFunction::hill_climb(const Mat& kappa, vector<Point>& pt1, vector<Point2d> pt1_, double lmd){
        //kappa_pad = np.pad(kappa, ((1,1),(1,1)),
        //    mode='constant', constant_values=-np.inf)
        size_t N = pt1.size();
        vector<bool> msk(N, true);
    
        Mat kappa_pad;

        copyMakeBorder(kappa,kappa_pad,1,1,1,1,BORDER_DEFAULT,Scalar(0));
      
        vector<int> patch{-1, 0, 1};
        #pragma omp parallel for
        for(int k=0; k<pt1.size(); k++){
            if(msk[k]){
                double F = static_cast<double>(kappa.at<uchar>(pt1[k].y, pt1[k].x));
                while(true){
                    vector<double> Fs;
                    vector<Point> ds;
                    for(int i = 0; i<patch.size(); i++){
                        int di = patch[i];
                        for(int j = 0; j<patch.size(); j++){
                            int dj = patch[j];
                            ds.push_back(Point(di,dj));
                            if(di==0 && dj==0){
                                Fs.push_back(F);
                                continue;
                            }

                            //double f;

                            Point2d pt = Point2d(Point(pt1[k])) + Point2d(di,dj) - pt1_[k];
                            double d_pt = sqrt(pow(pt.x, 2)+pow(pt.y, 2));

                            double f = static_cast<double>(kappa_pad.at<uchar>(pt1[k].y+(1+dj), pt1[k].x+(1+di))) + lmd * w_fn(d_pt, 0.1);
                            //f = f_;
                            Fs.push_back(f);
                        }
                    }
                    //colwise argmax
                    unsigned int sel;
                    double FsMax = -100000;
                    
                    for(int j=0; j<Fs.size(); j++){
                        if(FsMax < Fs[j]){
                            FsMax = Fs[j];
                            sel = j;
                        }
                    }
                    
                    F = FsMax;
                    
                    //update pt    
                    pt1[k] = pt1[k] + ds[sel];


                    //recalc msk
                    if(sel==4){
                        msk[k] = false;
                        break;
                    }
                    
                }
            }
        }
    }

    double VitamineFunction::p_fn(const double x, const double sigma)const{
        // really should be `rho`, but using p anyway
        // Geman-McClure Kernel
        double xsq = pow(x,2);
        double ssq = pow(sigma,2);
        return (xsq / (xsq + ssq));
    }
    double VitamineFunction::w_fn(const double x, const double sigma)const{
        return (1.0 - p_fn(x, sigma));
    }

}