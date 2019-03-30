
#include <list>
#include <mutex>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>

#include "SLAM/map.h"
#include "SLAM/frame.h"
#include "SLAM/key_frame_database.h"
#include "SLAM/slam_system.h"

namespace SLAM
{

class Map;
class SLAM_System;

class Tracker
{
public:

    cv::Mat imgGray_;
    Frame currentFrame_;

    std::vector<int> initLastMatches_;
    std::vector<int> initMatches_;
    std::vector<cv::Point2f> prevMatched_;
    std::vector<cv::Point3f> initPoint3d;
    Frame initialFrame;

    // Lists used to recover the full camera trajectory at the end of the execution.
    // Basically we store the reference keyframe for each frame and its relative transformation
    std::list<cv::Mat> relativeFramePoses;
    std::list<KeyFrame*> referenceFrames;
    std::list<double> frameTimes;
    std::list<bool> lost;

    void Reset();

    void Track();
    cv::Mat TrackImageMonocular(const cv::Mat& img, const double& timeStamp, const cv::Mat& poseTwc);

    void UpdateLocalMap();
    void UpdateLocalPoints();
    void UpdataLocalKeyFrames();

    bool TrackLocalMap();
    void SearchLocalPoints();

    bool NeedNewKeyFrame();
    void CreateNewKeyFrame();

public:
    // Tracking states
    enum TrackingState{
        SYSTEM_NOT_READY=-1,
        NO_IMAGES_YET=0,
        NOT_INITIALIZED=1,
        OK=2,
        LOST=3
    };

    TrackingState state_;
    TrackingState lastProcessedState_;

    cv::FeatureDetector pFeatureDetector_;

    KeyFrame* pReferenceKF_;
    std::vector<KeyFrame*> localKFs_;
    std::vector<MapPoint*> localMapPoints_;

    SLAM_System* pSystem_;

    Map* pMap_;

    cv::Mat K_;
    cv::Mat distortCoeff_;

    float depthmapFactor_; // 输入的深度图的 比例

    int32_t numOfMatchesInliers_;

    KeyFrame* pLastKeyFrame_;
    Frame lastFrame_;
    uint64_t lastKFId_;

    std::list<MapPoint*> temporalMapPoints_;
}







}

