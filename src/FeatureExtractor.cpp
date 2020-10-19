#include "FeatureExtractor.h"

namespace VITAMINE
{
    Mat FeatureExtractor::curvature(cv::Mat src){
        
        const int scale = 1;
        const int delta = 0;
        const int ddepth = CV_8U; //CV_64F
        const int ksize = 1; //5
 
        Mat fx, fy, fxx, fyy, fxy;
        cv::Sobel( src, fx, ddepth, 1, 0, ksize, scale, delta, BORDER_DEFAULT );
        cv::Sobel( src, fy, ddepth, 0, 1, ksize, scale, delta, BORDER_DEFAULT );
        cv::Sobel( src, fxx, ddepth, 2, 0, ksize, scale, delta, BORDER_DEFAULT );
        cv::Sobel( src, fyy, ddepth, 0, 2, ksize, scale, delta, BORDER_DEFAULT );
        cv::Sobel( src, fxy, ddepth, 1, 1, ksize, scale, delta, BORDER_DEFAULT );
 
        Mat k = (fy.mul(fy)).mul(fxx) - 2*(fx.mul(fy)).mul(fxy) + (fx.mul(fx)).mul(fyy);

        return k;
    }

    vector<Point> bhContoursCenter(const vector<vector<Point>>& contours, vector<KeyPoint>& keypoints, bool centerOfMass,int contourIdx=-1){
 
        if (contourIdx > -1){
            if (centerOfMass){
                Moments m = moments(contours[contourIdx],true);
                keypoints.push_back( KeyPoint(m.m10/m.m00, m.m01/m.m00, 1));
            }else{
                Rect rct = boundingRect(contours[contourIdx]);
                keypoints.push_back( KeyPoint(rct.x + rct.width / 2 ,rct.y + rct.height / 2, 1));
            }
        }else{
            if (centerOfMass){
                for (int i=0; i < contours.size();i++){
                    Moments m = moments(contours[i],true);
                    keypoints.push_back( KeyPoint(m.m10/m.m00, m.m01/m.m00, 1));

                }
            }else{
                for (int i=0; i < contours.size(); i++){
                    Rect rct = boundingRect(contours[i]);
                    keypoints.push_back(KeyPoint(rct.x + rct.width / 2 ,rct.y + rct.height / 2, 1));
                }
            }
        }
    }

    void FeatureExtractor::local_maxima(Mat img, vector<KeyPoint>& keypoints, int neighbor){
        //TODO: adjust using ED algorithm
        /* int wsize = 9;
        Mat ker = getStructuringElement(cv::MORPH_RECT, cv::Size(wsize, wsize));
        
        Mat imx;
        dilate(img, imx, ker);

        Mat msk = (img >= imx);

        Mat e_img;
        erode(img, e_img, ker);
        Mat flat_msk = (img > e_img);
        bitwise_and(msk,flat_msk,msk);

        Mat val_msk = (img > np.percentile(img, 95.0));
        bitwise_and(msk, val_msk, msk);
        
        idx = np.stack(np.nonzero(msk), axis=-1)
        return msk[...,None].astype(np.float32), idx */
    }

    void FeatureExtractor::detect(const Mat& image, vector<KeyPoint>& keypoints, Mat& descriptors)
    { 
        Mat kappa = curvature(image); //Done  
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