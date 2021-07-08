#include "FeatureExtractor.h"
#include <omp.h>
#include "QuadTree.h"

namespace VITAMINE
{
    //why?
    const int PATCH_SIZE = 31;
    const int HALF_PATCH_SIZE = 15;
    const int EDGE_THRESHOLD = 19;

    FeatureExtractor::FeatureExtractor(){

        extractor = ORB::create(500);


    }; //default constructor

    Mat FeatureExtractor::curvature(const cv::Mat& img){
        std::vector<cv::Mat> kappa(nlevels);
        Mat blur;
        GaussianBlur(img, blur, Size(3, 3), 0, 0, BORDER_DEFAULT);
        const int scale = 1;
        const int delta = 0;
        const int ddepth = CV_8U; //CV_64F
        const int ksize = 1;
        //GaussianBlur(src, src, Size(3, 3), 0, 0, BORDER_DEFAULT);
        Mat fx, fy, fxx, fyy, fxy;
        cv::Sobel( blur, fx, ddepth, 1, 0, ksize, scale, delta, BORDER_DEFAULT );
        cv::Sobel( blur, fy, ddepth, 0, 1, ksize, scale, delta, BORDER_DEFAULT );
        cv::Sobel( blur, fxx, ddepth, 2, 0, ksize, scale, delta, BORDER_DEFAULT );
        cv::Sobel( blur, fyy, ddepth, 0, 2, ksize, scale, delta, BORDER_DEFAULT );
        cv::Sobel( blur, fxy, ddepth, 1, 1, ksize, scale, delta, BORDER_DEFAULT );
 
        Mat k = (fy.mul(fy)).mul(fxx) - 2*(fx.mul(fy)).mul(fxy) + (fx.mul(fx)).mul(fyy);
  
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
        return pixel+1;
    }

    void non_maxima_suppression(const cv::Mat& image, cv::Mat& mask, bool remove_plateaus) {
    // find pixels that are equal to the local neighborhood not maximum (including 'plateaus')
    
    cv::dilate(image, mask, cv::Mat());
    cv::compare(image, mask, mask, cv::CMP_GE);

    // optionally filter out pixels that are equal to the local minimum ('plateaus')
    if (remove_plateaus) {
        cv::Mat non_plateau_mask;
        cv::erode(image, non_plateau_mask, cv::Mat());
        cv::compare(image, non_plateau_mask, non_plateau_mask, cv::CMP_GT);
        cv::bitwise_and(mask, non_plateau_mask, mask);
    }
}

    Mat FeatureExtractor::local_maxima(const Mat& img){
        
        //#pragma omp parallel for
        Mat th, msk;
        /* double maxVal = 0;
        minMaxLoc( src, 0, &maxVal, 0, 0);
        cout<<"maxVal:"<<maxVal<<endl; */
        threshold( img, th, 200, 255, THRESH_TOZERO );
        /* double min, max;
        minMaxIdx(src, &min, &max);
        normalize(src, src, 0, 0, NORM_MINMAX); */

        
        non_maxima_suppression(th, msk, 1);

        /* Mat val_msk = (img[0] >= percentile(img[0], 95.0));
        bitwise_and(pyramid_msk, val_msk, pyramid_msk); */

        return msk;
        
    }

    void FeatureExtractor::detect(const Mat& img, vector<KeyPoint>& keypoints, Mat& kappa)
    { 
        //ComputePyramid(image);
        cout<<"detect curvature"<<endl;
        kappa = curvature(img); //Done 
        //imshow("mCurrentFrame->kappa", kappa);
        
        cout<<"calculate local maxima"<<endl;
        Mat localPoints = local_maxima(kappa);
        
        /* vector<Point2f> corners;
        double qualityLevel = 0.01;
        double minDistance = 3;
        int blockSize = 3, gradientSize = 3;
        bool useHarrisDetector = false;
        double k = 0.04;
        goodFeaturesToTrack( image,
                            corners,
                            100000,
                            qualityLevel,
                            minDistance,
                            Mat(),
                            blockSize,
                            gradientSize,
                            useHarrisDetector,
                            k );
        for(int i=0; i<corners.size(); i++){
            keypoints.push_back(KeyPoint(corners[i].x, corners[i].y, 1));
        } */

        /* imshow("local points?: ", localPoints);
        waitKey(30); */
        /* QuadTree qt(localPoints, 100, 1); //int area, float threshold
        keypoints = qt.getKeyPoints();
        cout<<"keypoints num: "<<keypoints.size()<<endl; */
        //vector<KeyPoint> keypoints;
        
        Size imgsize = img.size();
        std::vector<const float*> tmpCorners;
        // collect list of pointers to features - put them into temporary image
        //Mat mask = _mask.getMat();
        for( int y = 10; y < imgsize.height - 10; y++ )
        {
            const float* eig_data = (const float*)localPoints.ptr(y);
            //const float* tmp_data = (const float*)tmp.ptr(y);
            //const uchar* mask_data = mask.data ? mask.ptr(y) : 0;

            for( int x = 10; x < imgsize.width - 10; x++ )
            {
                float val = eig_data[x];
                if( val != 0 )//&& val == tmp_data[x] && (!mask_data || mask_data[x]) )
                    tmpCorners.push_back(eig_data + x);
            }
        }

        size_t i, j, total = tmpCorners.size(), ncorners = 0;

        int minDistance = 10;
        
        // Partition the image into larger grids
        int w = img.cols;
        int h = img.rows;
        
        const int cell_size = cvRound(minDistance);
        const int grid_width = (w + cell_size ) / cell_size;
        const int grid_height = (h + cell_size ) / cell_size;

        std::vector<std::vector<Point2f> > grid(grid_width*grid_height);

        minDistance *= minDistance;
        
        for( i = 0; i < total; i++ )
        {
            
            int ofs = (int)((const uchar*)tmpCorners[i] - localPoints.ptr());
            int y = (int)(ofs / localPoints.step);
            int x = (int)((ofs - y*localPoints.step)/sizeof(uchar));
            
            bool good = true;

            int x_cell = x / cell_size;
            int y_cell = y / cell_size;

            int x1 = x_cell - 1;
            int y1 = y_cell - 1;
            int x2 = x_cell + 1;
            int y2 = y_cell + 1;

            // boundary check
            x1 = std::max(0, x1);
            y1 = std::max(0, y1);
            x2 = std::min(grid_width-1, x2);
            y2 = std::min(grid_height-1, y2);


            for( int yy = y1; yy <= y2; yy++ ){
                for( int xx = x1; xx <= x2; xx++ ){
                    std::vector <Point2f> &m = grid[yy*grid_width + xx];

                    if( m.size() ){
                        for(j = 0; j < m.size(); j++){
                            float dx = x - m[j].x;
                            float dy = y - m[j].y;

                            if( dx*dx + dy*dy < minDistance ){
                                good = false;
                                break;
                            }
                        }
                    }
                    if(!good) break;
                }
                if(!good) break;
            }
            if (good){
                
                grid[y_cell*grid_width + x_cell].push_back(Point2f((float)x, (float)y));

                keypoints.push_back(KeyPoint(x,y,1));
                ++ncorners;
            }
        }
        cout<<"point num:"<<ncorners<<endl;
    }
    void FeatureExtractor::detectFeatsForMotion(const Mat& img, vector<KeyPoint>& keypoints, Mat& descriptors){

        extractor->detectAndCompute(img, noArray(), keypoints, descriptors);
        
    }

    /* void FeatureExtractor::ComputePyramid(cv::Mat image){
        for (int level = 0; level < nlevels; ++level)
        {
            float scale = mvInvScaleFactor[level];
            Size sz(cvRound((float)image.cols*scale), cvRound((float)image.rows*scale));
            Size wholeSize(sz.width + EDGE_THRESHOLD*2, sz.height + EDGE_THRESHOLD*2);
            Mat temp(wholeSize, image.type()), masktemp;
            mvImagePyramid[level] = temp(Rect(EDGE_THRESHOLD, EDGE_THRESHOLD, sz.width, sz.height));

            // Compute the resized image
            if( level != 0 )
            {
                resize(mvImagePyramid[level-1], mvImagePyramid[level], sz, 0, 0, INTER_LINEAR);

                copyMakeBorder(mvImagePyramid[level], temp, EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD,
                            BORDER_REFLECT_101+BORDER_ISOLATED);            
            }
            else
            {
                copyMakeBorder(image, temp, EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD,
                            BORDER_REFLECT_101);            
            }
        }
    } */

}//namespace VITAMINE