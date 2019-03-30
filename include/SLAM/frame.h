
#include <vector>

#include <opencv2/core.hpp>

#include "SLAM/map_point.h"
#include "SLAM/key_frame.h"

namespace SLAM
{

class Frame
{
public:
    Frame() {}

    Frame(const Frame& frame);

    Frame(const cv::Mat& imgGray, const double& timeStamp, cv::Mat& K, cv::Mat& distortCoeff);

    Frame(const cv::Mat& imgGray, const cv::Mat& imgDepth, const double& timeStamp,
          cv::Mat& K, cv::Mat& distortCoeff);

    void SetPose(cv::Mat Tcw);

    void UpdatePoseMatrices();

    inline cv::Mat GetCameraCenter()  {   return posetwc_.clone();    }

    bool IsInFrustum(MapPoint* pMapPoint, float viewingCosLimit);
    cv::Mat UnprojectStereo(const int& i);

    //
    void UndistortKeyPoints();
    void ComputeImageBounds(const cv::Mat& img);


public:

    double timeStamp_;
    cv::Mat K_;
    static float fx_;
    static float fy_;
    static float cx_;
    static float cy_;
    static float invFx_;
    static float invFy_;
    cv::Mat distortionCoef_;

    int32_t numOfKeyPoints_;

    // 观察到的关键点
    std::vector<cv::KeyPoint> keyPoints_;
    std::vector<cv::KeyPoint> keyPointsUndistorted_;
    // 关键点对应的深度
    std::vector<float> depths_;

    // 描述子
    cv::Mat descriptors_;

    // 关键点对应的地图点，如果没有就是NULL
    std::vector<MapPoint*> mapPoints_;

    std::vector<bool> outliers_;



    // 当前和下一个的 frame id
    static uint64_t nextId_;
    uint64_t id_;

    // 参考关键帧
    KeyFrame* pRefKF_;


    // 跟位姿相关的量
    cv::Mat poseTcw_;
    cv::Mat poseRcw_;
    cv::Mat posetcw_;
    cv::Mat poseRwc_;
    cv::Mat posetwc_;
}

}
