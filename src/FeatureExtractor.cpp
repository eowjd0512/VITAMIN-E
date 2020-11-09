#include "FeatureExtractor.h"

namespace VITAMINE
{
    //why?
    const int PATCH_SIZE = 31;
    const int HALF_PATCH_SIZE = 15;
    const int EDGE_THRESHOLD = 19;

    Mat FeatureExtractor::curvature(cv::Mat src){
        //GaussianBlur(src, src, Size(3, 3), 0, 0, BORDER_DEFAULT);
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
    Mat FeatureExtractor::local_maxima(Mat img){
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
        
        Mat val_msk = (img >= percentile(img, 99.0));
        bitwise_and(msk, val_msk, msk);
        
        return msk;
        
    }

    void ExtractorNode::DivideNode(ExtractorNode &n1, ExtractorNode &n2, ExtractorNode &n3, ExtractorNode &n4)
    {
        const int halfX = ceil(static_cast<float>(UR.x-UL.x)/2);
        const int halfY = ceil(static_cast<float>(BR.y-UL.y)/2);

        //Define boundaries of childs
        n1.UL = UL;
        n1.UR = cv::Point2i(UL.x+halfX,UL.y);
        n1.BL = cv::Point2i(UL.x,UL.y+halfY);
        n1.BR = cv::Point2i(UL.x+halfX,UL.y+halfY);
        n1.vKeys.reserve(vKeys.size());

        n2.UL = n1.UR;
        n2.UR = UR;
        n2.BL = n1.BR;
        n2.BR = cv::Point2i(UR.x,UL.y+halfY);
        n2.vKeys.reserve(vKeys.size());

        n3.UL = n1.BL;
        n3.UR = n1.BR;
        n3.BL = BL;
        n3.BR = cv::Point2i(n1.BR.x,BL.y);
        n3.vKeys.reserve(vKeys.size());

        n4.UL = n3.UR;
        n4.UR = n2.BR;
        n4.BL = n3.BR;
        n4.BR = BR;
        n4.vKeys.reserve(vKeys.size());

        //Associate points to childs
        for(size_t i=0;i<vKeys.size();i++)
        {
            const cv::KeyPoint &kp = vKeys[i];
            if(kp.pt.x<n1.UR.x)
            {
                if(kp.pt.y<n1.BR.y)
                    n1.vKeys.push_back(kp);
                else
                    n3.vKeys.push_back(kp);
            }
            else if(kp.pt.y<n1.BR.y)
                n2.vKeys.push_back(kp);
            else
                n4.vKeys.push_back(kp);
        }

        if(n1.vKeys.size()==1)
            n1.bNoMore = true;
        if(n2.vKeys.size()==1)
            n2.bNoMore = true;
        if(n3.vKeys.size()==1)
            n3.bNoMore = true;
        if(n4.vKeys.size()==1)
            n4.bNoMore = true;

    }
    std::vector<cv::KeyPoint> FeatureExtractor::ComputeKeyPointsOctTree(const Mat& localPoints){
        
        vector<KeyPoint> keypoints;
        uchar* localPoints_ptr = (uchar*)localPoints.data;
        /* for(int i=0; i<localPoints.rows;i++){
            for(int j=0; j<localPoints.cols;j++){
                if(msk_[i * localPoints.cols + j] > 0){
                    keypoints.push_back(KeyPoint(j, i, 1));                
                }
            }
        } */

        //allKeypoints.resize(nlevels);

        const float W = 30;

        //for (int level = 0; level < nlevels; ++level)
        //{
        const int minBorderX = EDGE_THRESHOLD-3;
        const int minBorderY = minBorderX;
        const int maxBorderX = localPoints.cols-EDGE_THRESHOLD+3;
        const int maxBorderY = localPoints.rows-EDGE_THRESHOLD+3;

        vector<cv::KeyPoint> vToDistributeKeys;
        vToDistributeKeys.reserve(nfeatures*10);

        const float width = (maxBorderX-minBorderX);
        const float height = (maxBorderY-minBorderY);

        const int nCols = width/W;
        const int nRows = height/W;
        const int wCell = ceil(width/nCols);
        const int hCell = ceil(height/nRows);

        for(int i=0; i<nRows; i++)
        {
            const float iniY =minBorderY+i*hCell;
            float maxY = iniY+hCell+6;

            if(iniY>=maxBorderY-3)
                continue;
            if(maxY>maxBorderY)
                maxY = maxBorderY;

            for(int j=0; j<nCols; j++)
            {
                const float iniX =minBorderX+j*wCell;
                float maxX = iniX+wCell+6;
                if(iniX>=maxBorderX-6)
                    continue;
                if(maxX>maxBorderX)
                    maxX = maxBorderX;

                vector<cv::KeyPoint> vKeysCell;

                for(int y=iniY; y< maxY;y++){
                    for(int x=iniX; x< maxX;x++){
                        if(localPoints_ptr[y * localPoints.cols + x] > 0){
                            vKeysCell.push_back(KeyPoint(x, y, 1));                
                        }
                    }
                }
                /* FAST(mvImagePyramid[level].rowRange(iniY,maxY).colRange(iniX,maxX),
                    vKeysCell,iniThFAST,true); */

               /*  if(vKeysCell.empty())
                {
                    FAST(mvImagePyramid[level].rowRange(iniY,maxY).colRange(iniX,maxX),
                        vKeysCell,minThFAST,true);
                } */

                if(!vKeysCell.empty())
                {
                    for(vector<cv::KeyPoint>::iterator vit=vKeysCell.begin(); vit!=vKeysCell.end();vit++)
                    {
                        //(*vit).pt.x+=j*wCell;
                        //(*vit).pt.y+=i*hCell;
                        (*vit).pt.x-=minBorderX;
                        (*vit).pt.y-=minBorderY;
                        vToDistributeKeys.push_back(*vit);
                    }
                }

            }
        }

        //vector<KeyPoint> & keypoints = allKeypoints[level];
        keypoints.reserve(nfeatures);

        keypoints = DistributeOctTree(vToDistributeKeys, minBorderX, maxBorderX,
                                    minBorderY, maxBorderY, nfeatures, 1);

        const int scaledPatchSize = PATCH_SIZE;

        // Add border to coordinates and scale information
        const int nkps = keypoints.size();

        for(int i=0; i<nkps ; i++)
        {
            keypoints[i].pt.x+=minBorderX;
            keypoints[i].pt.y+=minBorderY;
            keypoints[i].octave=1;
            keypoints[i].size = scaledPatchSize;
        }
        //}
        return keypoints;
    }
    std::vector<cv::KeyPoint> FeatureExtractor::DistributeOctTree(const std::vector<cv::KeyPoint>& vToDistributeKeys, const int &minX,
                                           const int &maxX, const int &minY, const int &maxY, const int &N, const int &level){

        // Compute how many initial nodes   
        const int nIni = round(static_cast<float>(maxX-minX)/(maxY-minY));
        
        const float hX = static_cast<float>(maxX-minX)/nIni;

        list<ExtractorNode> lNodes;

        vector<ExtractorNode*> vpIniNodes;
        vpIniNodes.resize(nIni);

        for(int i=0; i<nIni; i++)
        {
            ExtractorNode ni;
            ni.UL = cv::Point2i(hX*static_cast<float>(i),0);
            ni.UR = cv::Point2i(hX*static_cast<float>(i+1),0);
            ni.BL = cv::Point2i(ni.UL.x,maxY-minY);
            ni.BR = cv::Point2i(ni.UR.x,maxY-minY);
            ni.vKeys.reserve(vToDistributeKeys.size());

            lNodes.push_back(ni);
            vpIniNodes[i] = &lNodes.back();
        }

        //Associate points to childs
        for(size_t i=0;i<vToDistributeKeys.size();i++)
        {
            const cv::KeyPoint &kp = vToDistributeKeys[i];
            vpIniNodes[kp.pt.x/hX]->vKeys.push_back(kp);
        }
        
        list<ExtractorNode>::iterator lit = lNodes.begin();

        while(lit!=lNodes.end())
        {
            if(lit->vKeys.size()==1)
            {
                lit->bNoMore=true;
                lit++;
            }
            else if(lit->vKeys.empty())
                lit = lNodes.erase(lit);
            else
                lit++;
        }

        bool bFinish = false;

        int iteration = 0;

        vector<pair<int,ExtractorNode*> > vSizeAndPointerToNode;
        vSizeAndPointerToNode.reserve(lNodes.size()*4);

        while(!bFinish)
        {
            iteration++;

            int prevSize = lNodes.size();

            lit = lNodes.begin();

            int nToExpand = 0;

            vSizeAndPointerToNode.clear();

            while(lit!=lNodes.end())
            {
                if(lit->bNoMore)
                {
                    // If node only contains one point do not subdivide and continue
                    lit++;
                    continue;
                }
                else
                {
                    // If more than one point, subdivide
                    ExtractorNode n1,n2,n3,n4;
                    lit->DivideNode(n1,n2,n3,n4);

                    // Add childs if they contain points
                    if(n1.vKeys.size()>0)
                    {
                        lNodes.push_front(n1);                    
                        if(n1.vKeys.size()>1)
                        {
                            nToExpand++;
                            vSizeAndPointerToNode.push_back(make_pair(n1.vKeys.size(),&lNodes.front()));
                            lNodes.front().lit = lNodes.begin();
                        }
                    }
                    if(n2.vKeys.size()>0)
                    {
                        lNodes.push_front(n2);
                        if(n2.vKeys.size()>1)
                        {
                            nToExpand++;
                            vSizeAndPointerToNode.push_back(make_pair(n2.vKeys.size(),&lNodes.front()));
                            lNodes.front().lit = lNodes.begin();
                        }
                    }
                    if(n3.vKeys.size()>0)
                    {
                        lNodes.push_front(n3);
                        if(n3.vKeys.size()>1)
                        {
                            nToExpand++;
                            vSizeAndPointerToNode.push_back(make_pair(n3.vKeys.size(),&lNodes.front()));
                            lNodes.front().lit = lNodes.begin();
                        }
                    }
                    if(n4.vKeys.size()>0)
                    {
                        lNodes.push_front(n4);
                        if(n4.vKeys.size()>1)
                        {
                            nToExpand++;
                            vSizeAndPointerToNode.push_back(make_pair(n4.vKeys.size(),&lNodes.front()));
                            lNodes.front().lit = lNodes.begin();
                        }
                    }

                    lit=lNodes.erase(lit);
                    continue;
                }
            }       

            // Finish if there are more nodes than required features
            // or all nodes contain just one point
            if((int)lNodes.size()>=N || (int)lNodes.size()==prevSize)
            {
                bFinish = true;
            }
            else if(((int)lNodes.size()+nToExpand*3)>N)
            {

                while(!bFinish)
                {

                    prevSize = lNodes.size();

                    vector<pair<int,ExtractorNode*> > vPrevSizeAndPointerToNode = vSizeAndPointerToNode;
                    vSizeAndPointerToNode.clear();

                    sort(vPrevSizeAndPointerToNode.begin(),vPrevSizeAndPointerToNode.end());
                    for(int j=vPrevSizeAndPointerToNode.size()-1;j>=0;j--)
                    {
                        ExtractorNode n1,n2,n3,n4;
                        vPrevSizeAndPointerToNode[j].second->DivideNode(n1,n2,n3,n4);

                        // Add childs if they contain points
                        if(n1.vKeys.size()>0)
                        {
                            lNodes.push_front(n1);
                            if(n1.vKeys.size()>1)
                            {
                                vSizeAndPointerToNode.push_back(make_pair(n1.vKeys.size(),&lNodes.front()));
                                lNodes.front().lit = lNodes.begin();
                            }
                        }
                        if(n2.vKeys.size()>0)
                        {
                            lNodes.push_front(n2);
                            if(n2.vKeys.size()>1)
                            {
                                vSizeAndPointerToNode.push_back(make_pair(n2.vKeys.size(),&lNodes.front()));
                                lNodes.front().lit = lNodes.begin();
                            }
                        }
                        if(n3.vKeys.size()>0)
                        {
                            lNodes.push_front(n3);
                            if(n3.vKeys.size()>1)
                            {
                                vSizeAndPointerToNode.push_back(make_pair(n3.vKeys.size(),&lNodes.front()));
                                lNodes.front().lit = lNodes.begin();
                            }
                        }
                        if(n4.vKeys.size()>0)
                        {
                            lNodes.push_front(n4);
                            if(n4.vKeys.size()>1)
                            {
                                vSizeAndPointerToNode.push_back(make_pair(n4.vKeys.size(),&lNodes.front()));
                                lNodes.front().lit = lNodes.begin();
                            }
                        }

                        lNodes.erase(vPrevSizeAndPointerToNode[j].second->lit);

                        if((int)lNodes.size()>=N)
                            break;
                    }

                    if((int)lNodes.size()>=N || (int)lNodes.size()==prevSize)
                        bFinish = true;

                }
            }
        }

        // Retain the best point in each node
        vector<cv::KeyPoint> vResultKeys;
        vResultKeys.reserve(nfeatures);
        for(list<ExtractorNode>::iterator lit=lNodes.begin(); lit!=lNodes.end(); lit++)
        {
            vector<cv::KeyPoint> &vNodeKeys = lit->vKeys;
            cv::KeyPoint* pKP = &vNodeKeys[0];
            float maxResponse = pKP->response;

            for(size_t k=1;k<vNodeKeys.size();k++)
            {
                if(vNodeKeys[k].response>maxResponse)
                {
                    pKP = &vNodeKeys[k];
                    maxResponse = vNodeKeys[k].response;
                }
            }

            vResultKeys.push_back(*pKP);
        }

        return vResultKeys;
    }

    void FeatureExtractor::detect(const Mat& image, vector<KeyPoint>& keypoints, Mat& descriptors, Mat& kappa)
    { 
        //vector<KeyPoint> allKeypoints;
        kappa = curvature(image); //Done  
        Mat localPoints = local_maxima(kappa);
        //vector<KeyPoint> keypoints;
        uchar* localPoints_ptr = (uchar*)localPoints.data;
        for(int i=0; i<localPoints.rows;i++){
            for(int j=0; j<localPoints.cols;j++){
                if(localPoints_ptr[i * localPoints.cols + j] > 0){
                    keypoints.push_back(KeyPoint(j, i, 1));                
                }
            }
        }
        //keypoints = ComputeKeyPointsOctTree(localPoints);
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