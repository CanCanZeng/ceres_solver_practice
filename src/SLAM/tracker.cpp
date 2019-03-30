#include "SLAM/tracker.h"

namespace SLAM {

cv::Mat Tracker::TrackImageMonocular(const cv::Mat &img, const double &timeStamp, const cv::Mat &poseTwc)
{
    imgGray_ = img.clone();
    if(imgGray_.channels() == 3)
    {
        cv::cvtColor(imgGray_, imgGray_, CV_BGR2GRAY);
    }
    else if(imgGray_.channels() == 4)
    {
        cv::cvtColor(imgGray_, imgGray_, CV_BGRA2GRAY);
    }

    currentFrame_ = Frame(imgGray_, timeStamp, K_, distortCoeff_);

    Track();

    return currentFrame_.poseTcw_.clone();
}

void Tracker::Track()
{
    if(state_ == NO_IMAGES_YET)
        state_ = NOT_INITIALIZED;

    lastProcessedState_ = state_;

    // 跟踪的时候不允许地图更新
    std::unique_lock<std::mutex> lock(pMap_->mutexMapUpdate_);

    if(state_ == NOT_INITIALIZED)
    {

    }

}

}
