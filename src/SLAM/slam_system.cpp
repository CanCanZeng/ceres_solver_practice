#include "SLAM/slam_system.h"

namespace SLAM {

cv::Mat SLAM_System::TrackMonocular(const cv::Mat &img, const double &timeStamp, const cv::Mat &poseTwc)
{
    // check reset
    {
        if(shouldReset_)
        {
            pTracker_->Reset();
            shouldReset_ = false;
        }
    }

    cv::Mat Tcw = pTracker_->TrackImageMonocular(img, timeStamp, poseTwc);



    return Tcw;
}



}
