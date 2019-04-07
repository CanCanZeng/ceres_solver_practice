#ifndef SLAM_SYSTEM_H
#define SLAM_SYSTEM_H

#include <string>
#include <thread>
#include <unistd.h>
#include <atomic>

#include <opencv2/core.hpp>

#include "SLAM/map.h"
#include "SLAM/frame_database.h"
#include "SLAM/tracker.h"
#include "SLAM/optimizer.h"

namespace SLAM
{

class SLAM_System
{
public:
    SLAM_System();
    ~SLAM_System();

    void TrackMonocular(const cv::Mat& img, const Eigen::Matrix3f& Rwc, const Eigen::Vector3f& twc, const Eigen::Vector4f& intrin);
    void CompleteTracking();

    // 跟踪结束了，进行优化
    void Optimize();

    void SaveMapPoints(std::string plyName);

    void Reset();

    void ShutDown();

    // 最近一帧的跟踪情况
    std::vector<MapPoint*> GetTrackedMapPoints();
    std::vector<cv::KeyPoint> GetTrackedKeyPointsUndistorted();

public:
    FrameDatabase* pFrameDatabase_ = nullptr;
    Map* pMap_ = nullptr;
    Tracker* pTracker_ = nullptr;
    Optimizer* pOptimizer_ = nullptr;

    std::atomic_bool shouldReset_;
    std::vector<MapPoint*> trackedMapPoints_;
    std::vector<cv::KeyPoint> trackedKeyPointsUndistorted_;
    std::mutex mutexState_;
};

SLAM_System::SLAM_System()
{
    pFrameDatabase_ = new FrameDatabase;
    pMap_ = new Map;
    pTracker_ = new Tracker;
    pOptimizer_ = new Optimizer;

    pMap_->pFrameDatabase_ = pFrameDatabase_;
    pTracker_->pFrameDatabase_ = pFrameDatabase_;
    pTracker_->pMap_ = pMap_;
}

SLAM_System::~SLAM_System()
{
    if(pFrameDatabase_ != nullptr)
    {
        delete pFrameDatabase_;
        pFrameDatabase_ = nullptr;
    }

    if(pMap_ != nullptr)
    {
        delete pMap_;
        pMap_ = nullptr;
    }

    if(pTracker_ != nullptr)
    {
        delete pTracker_;
        pTracker_ = nullptr;
    }

    if(pOptimizer_ != nullptr)
    {
        delete pOptimizer_;
        pOptimizer_ = nullptr;
    }
}

void SLAM_System::TrackMonocular(const cv::Mat &img, const Eigen::Matrix3f &Rwc, const Eigen::Vector3f &twc, const Eigen::Vector4f &intrin)
{
    pTracker_->TrackImageMonocular(img, Rwc, twc, intrin);
}

void SLAM_System::CompleteTracking()
{
    pTracker_->CreateLoop();
    pMap_->ArrageMapPoints();
}

void SLAM_System::Optimize()
{
    // 下面进行优化
    pOptimizer_->BuildOptimizationProblem(*pMap_);
    pOptimizer_->SolveProblem();
    pOptimizer_->GetResult(*pMap_);

}

void SLAM_System::SaveMapPoints(std::string plyName)
{
    pMap_->SaveMapPoints(plyName);
}

}  // namespace SLAM

#endif // SLAM_SYSTEM_H
