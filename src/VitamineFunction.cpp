#include "VitamineFunction.h"

namespace VITAMINE{
    const void VitamineFunction::setInitialFeatures(){
        std::vector<KeyPoint>& KeyPoints = mPrevFrame->mvKeysUn;
        
        for(int i=0; i< KeyPoints.size(); i++){
            TrackedFeature* tf_ = new TrackedFeature();
            tf_->pt = KeyPoints[i].pt;

            tf.push_back(tf_);
        }
    }
    const void VitamineFunction::addResidualFeatures(){

    }
    const void VitamineFunction::drawTrackingFeatures(){
        Mat src = mCurrentFrame->img;
        cvtColor(src,src,COLOR_GRAY2BGR);
        
        RNG rng(12345);
        
        for (int i = 0; i < tf.size(); i++) {
            Scalar color = Scalar(rng.uniform(0,255), rng.uniform(0, 255), rng.uniform(0, 255));
            circle(src, tf[i]->pt, 1, color, -1, 8, 0);
        }
        imshow("src",src);
        waitKey(30);
    }

    const void VitamineFunction::loadConsecutiveFrames(Frame* prevFrame, Frame* currentFrame){
        mPrevFrame = prevFrame;
        mCurrentFrame = currentFrame;
    }
    const void VitamineFunction::getDominantMotion(const std::vector<DMatch>& good_matches){
        int match_size = good_matches.size();
        
        vector<Point> pt0(match_size), pt1(match_size);
        
        /* std::transform(good_matches.begin(), good_matches.end(), pt0.begin(), 
        [mPrevFrame.mvKeysUn](DMatch matches) {return mPrevFrame.mvKeysUn[matches.queryIdx];});
        std::transform(good_matches.begin(), good_matches.end(), pt1.begin(), 
        [mCurrentFrame.mvKeysUn](DMatch matches) {return mCurrentFrame.mvKeysUn[matches.trainIdx];}); */
        vector<KeyPoint>& mPrevKP = mPrevFrame->mvKeysUn;
        vector<KeyPoint>& mCurrentKP = mCurrentFrame->mvKeysUn;

        for(int i=0; i<match_size; i++){
            pt0[i] = mPrevKP[good_matches[i].queryIdx].pt;
            pt1[i] = mCurrentKP[good_matches[i].trainIdx].pt;
        }

        Mat mpt0(pt0);
        Mat mpt1(pt1);
        
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
                tf[i] = static_cast<TrackedFeature*>(NULL);
                tf.erase(tf.begin()+i);
                i--;
                continue;
            }

            predictedP_precision.push_back(pred_pt);
            predictedP.push_back(Point(pred_pt));
            //predictedP2KeyPoint_idx.push_back(i);
        }
 
        const Mat& kappa = mCurrentFrame->kappa;

        double lmd=0.001;
        hill_climb(kappa, predictedP, predictedP_precision, lmd);

        //update pt
        assert(predictedP.size() == tf.size());
        for(int i=0; i<tf.size(); i++){
            tf[i]->pt = predictedP[i];
        }
      
    }

    void VitamineFunction::hill_climb(const Mat& kappa, vector<Point>& pt1, vector<Point2d> pt1_, double lmd){
        //kappa_pad = np.pad(kappa, ((1,1),(1,1)),
        //    mode='constant', constant_values=-np.inf)
        size_t N = pt1.size();
        vector<bool> msk(N, true);
        
        vector<double> F(N);

        for(int i=0; i<N; i++){
            F[i] = static_cast<double>(kappa.at<uchar>(pt1[i]));
        }   
        
        Mat kappa_pad;
        copyMakeBorder(kappa,kappa_pad,1,1,1,1,BORDER_CONSTANT,Scalar(0));

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
                    for(int k=0; k<pt1.size(); k++){
                        if(msk[k]){
                            Point2d pt = Point2d(Point(pt1[k])) + Point2d(di,dj) - pt1_[k];
                            double d_pt = sqrt(pow(pt.x, 2)+pow(pt.y, 2));
                            double f_ = static_cast<double>(kappa_pad.at<uchar>(Point(pt1[k].x+(1+di), pt1[k].y+(1+dj)))) + lmd * w_fn(d_pt, 0.1);
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
            //cout<<numRest<<endl;
            if (!numRest) break;
        }
        /* for(int i=0;i<N;i++){
            cout<<Point(pt1_[i])<<" -> "<<pt1[i]<<endl;
        } */
    }

    double VitamineFunction::p_fn(const double x, const double sigma){
        // really should be `rho`, but using p anyway
        // Geman-McClure Kernel
        double xsq = pow(x,2);
        double ssq = pow(sigma,2);
        return (xsq / (xsq + ssq));
    }
    double VitamineFunction::w_fn(const double x, const double sigma){
        return (1.0 - p_fn(x, sigma));
    }
    
    
    

}