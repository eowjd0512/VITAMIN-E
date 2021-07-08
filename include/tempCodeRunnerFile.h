void static BundleAdjustment(const std::vector<Frame*> &vpF, const std::vector<MapPoint*> &vpMP,
                                int nIterations = 5, bool *pbStopFlag=NULL, const unsigned long nLoopKF=0,
                                const bool bRobust = true);
    void static GlobalBundleAdjustemnt(Map* pMap, int nIterations=5, bool *pbStopFlag=NULL,
                                       const unsigned long nLoopKF=0, const bool bRobust = true);
    