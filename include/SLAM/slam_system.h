#include <string>
#include <thread>
#include <unistd.h>
#include <atomic>

#include <opencv2/core.hpp>

#include "SLAM/map.h"
#include "SLAM/key_frame_database.h"
#include "SLAM/tracker.h"

namespace SLAM
{

class Map;
class KeyFrameDatabase;
class Tracker;

class SLAM_System
{
    cv::Mat TrackRGBD(const cv::Mat& img, const cv::Mat& depthmap, const double& timeStamp);

    cv::Mat TrackMonocular(const cv::Mat& img, const double& timeStamp, const cv::Mat& poseTwc);

    void Reset();

    void ShutDown();

    // 最近一帧的跟踪情况
    std::vector<MapPoint*> GetTrackedMapPoints();
    std::vector<cv::KeyPoint> GetTrackedKeyPointsUndistorted();

public:
    KeyFrameDatabase* pKeyFrameDatabase_;
    Map* pMap_;
    Tracker* pTracker_;

    std::atomic_bool shouldReset_;
    std::vector<MapPoint*> trackedMapPoints_;
    std::vector<cv::KeyPoint> trackedKeyPointsUndistorted_;
    std::mutex mutexState_;
};




}

