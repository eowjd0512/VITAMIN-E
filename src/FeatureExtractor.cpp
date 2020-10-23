#include "FeatureExtractor.h"

namespace VITAMINE
{
    Mat FeatureExtractor::curvature(cv::Mat src){
        
        const int scale = 1;
        const int delta = 0;
        const int ddepth = CV_8U; //CV_64F
        const int ksize = 1;
        //GaussianBlur(src, src, Size(3, 3), 0, 0, BORDER_DEFAULT);
        Mat fx, fy, fxx, fyy, fxy;
        cv::Sobel( src, fx, ddepth, 1, 0, ksize, scale, delta, BORDER_DEFAULT );
        cv::Sobel( src, fy, ddepth, 0, 1, ksize, scale, delta, BORDER_DEFAULT );
        cv::Sobel( src, fxx, ddepth, 2, 0, ksize, scale, delta, BORDER_DEFAULT );
        cv::Sobel( src, fyy, ddepth, 0, 2, ksize, scale, delta, BORDER_DEFAULT );
        cv::Sobel( src, fxy, ddepth, 1, 1, ksize, scale, delta, BORDER_DEFAULT );
 
        Mat k = (fy.mul(fy)).mul(fxx) - 2*(fx.mul(fy)).mul(fxy) + (fx.mul(fx)).mul(fyy);
        //cout<<sqrt(sum(k.mul(k))[0])<<endl;
        //k /= sqrt(sum(k.mul(k))[0]); //l2norm
        GaussianBlur(k, k, Size(3, 3), 0, 0, BORDER_DEFAULT);
        //double min, max;
        //minMaxIdx(k, &min, &max);
        //normalize(k, k, 0, 1, NORM_MINMAX);
        
        return k;
    }
    size_t percentile(const Mat& img, float percent){
        // calculate histogram for every pixel value (i.e [0 - 255])
        cv::Mat hist;
        int histSize = 255;
        float range[] = { 1, 256 } ;
        const float* histRange = { range };
        bool uniform = true; bool accumulate = false;
        cv::calcHist( &img, 1, 0, cv::Mat(), hist, 1, &histSize, &histRange, uniform, accumulate );

        // total pixels in image
        float totalPixels =  countNonZero(img);

        // calculate percentage of every histogram bin (i.e: pixel value [0 - 255])
        // the 'bins' variable holds pairs of (int pixelValue, float percentage) 
        std::vector<std::pair<int, float>> bins;
        float percentage;
        for(int i = 0; i < 255; ++i)
        {
            percentage = (hist.at<float>(i,0)*100.0)/totalPixels;
            bins.push_back(std::make_pair(i, percentage));
        }

        // sort the bins according to percentage
        sort(bins.begin(), bins.end());

        // compute percentile for a pixel value
        int pixel = 0;
        float sum = 0;

        for (auto b : bins){
            sum += b.second;
            if (sum >= percent){
                pixel = b.first;
                break;
            }
  
        }
        //std::cout<<"Percentile for pixel("<<pixel+1<<"): "<<percent<<std::endl;
        return pixel+1;
    }
    void FeatureExtractor::local_maxima(Mat img, vector<KeyPoint>& keypoints){
        //TODO: adjust using ED algorithm
        int wsize = 9;
        Mat ker = getStructuringElement(cv::MORPH_RECT, cv::Size(wsize, wsize));
        
        Mat imx;
        dilate(img, imx, ker);
        
        Mat msk = (img >= imx);
        
        Mat e_img;
        erode(img, e_img, ker);

        Mat flat_msk = (img > e_img);

        bitwise_and(msk,flat_msk,msk);
        
        Mat val_msk = (img >= percentile(img, 90.0));
        bitwise_and(msk, val_msk, msk);
        
        uchar* msk_ = (uchar*)msk.data;
        for(int i=0; i<msk.rows;i++){
            for(int j=0; j<msk.cols;j++){
                if(msk_[i * msk.cols + j] > 0){
                    keypoints.push_back(KeyPoint(j, i, 1));
                    //circle(src, Point(j, i), 1.0, Scalar(255), 1, -1); 
                }
            }
        }
        //imshow("src", src);
        //imshow("imx", imx);
        //imshow("e_img", e_img);
        //imshow("msk", msk);
        //waitKey(30);
        //cout<<countNonZero(msk)<<" "<<keypoints.size()<<endl;
    }

    void FeatureExtractor::detect(const Mat& image, vector<KeyPoint>& keypoints, Mat& descriptors, Mat& kappa)
    { 
        kappa = curvature(image); //Done  
        local_maxima(kappa, keypoints);
        extractor->compute(image, keypoints, descriptors);
    }
    void FeatureExtractor::detectORB(const cv::Mat& image, const cv::Mat& mask, vector<KeyPoint>& keypoints,
                        cv::Mat& descriptors)
    { 
        
        /* 
        //ORB feature

        if(_image.empty())
            return;

        Mat image = _image.getMat();
        assert(image.type() == CV_8UC1 );

        // Pre-compute the scale pyramid
        ComputePyramid(image);

        vector < vector<KeyPoint> > allKeypoints;
        ComputeKeyPointsOctTree(allKeypoints);
        //ComputeKeyPointsOld(allKeypoints);

        Mat descriptors;

        int nkeypoints = 0;
        for (int level = 0; level < nlevels; ++level)
            nkeypoints += (int)allKeypoints[level].size();
        if( nkeypoints == 0 )
            _descriptors.release();
        else
        {
            _descriptors.create(nkeypoints, 32, CV_8U);
            descriptors = _descriptors.getMat();
        }

        _keypoints.clear();
        _keypoints.reserve(nkeypoints);

        int offset = 0;
        for (int level = 0; level < nlevels; ++level)
        {
            vector<KeyPoint>& keypoints = allKeypoints[level];
            int nkeypointsLevel = (int)keypoints.size();

            if(nkeypointsLevel==0)
                continue;

            // preprocess the resized image
            Mat workingMat = mvImagePyramid[level].clone();
            GaussianBlur(workingMat, workingMat, Size(7, 7), 2, 2, BORDER_REFLECT_101);

            // Compute the descriptors
            Mat desc = descriptors.rowRange(offset, offset + nkeypointsLevel);
            computeDescriptors(workingMat, keypoints, desc, pattern);

            offset += nkeypointsLevel;

            // Scale keypoint coordinates
            if (level != 0)
            {
                float scale = mvScaleFactor[level]; //getScale(level, firstLevel, scaleFactor);
                for (vector<KeyPoint>::iterator keypoint = keypoints.begin(),
                    keypointEnd = keypoints.end(); keypoint != keypointEnd; ++keypoint)
                    keypoint->pt *= scale;
            }
            // And add the keypoints to the output
            _keypoints.insert(_keypoints.end(), keypoints.begin(), keypoints.end());
        } */
    }
}//namespace VITAMINE