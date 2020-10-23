Mat pred_pt_ = Ab.colRange(0,2)*pt.t()+Ab.col(2);
            Point pred_pt(pred_pt_);

            if(pred_pt.x < mPrevFrame->mnMinX || pred_pt.x > mPrevFrame->mnMaxX || 
              pred_pt.y < mPrevFrame->mnMinY || pred_pt.y > mPrevFrame->mnMaxY)
                continue;

            trackedPrevP[i] = pred_pt;