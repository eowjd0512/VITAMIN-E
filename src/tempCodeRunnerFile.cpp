if(mImGray.channels()==3)
        {
            if(mbRGB)
                cvtColor(mImGray,mImGray,cv::COLOR_RGB2GRAY);
            else
                cvtColor(mImGray,mImGray,cv::COLOR_BGR2GRAY);
        }
        else if(mImGray.channels()==4)
        {
            if(mbRGB)
                cvtColor(mImGray,mImGray,cv::COLOR_RGBA2GRAY);
            else
                cvtColor(mImGray,mImGray,cv::COLOR_BGRA2GRAY);
        }