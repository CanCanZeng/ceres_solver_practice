
#include <map>

#include <opencv2/core.hpp>
#include <mutex>

#include "SLAM/key_frame.h"
#include "SLAM/frame.h"
#include "SLAM/map.h"

namespace SLAM
{
class KeyFrame;
class Map;
class Frame;

class MapPoint
{
public:
    MapPoint(const cv::Mat& pos, KeyFrame* pRefKF, Map* pMap);
    MapPoint(const cv::Mat& pos, Map* pMap, Frame* pFrame, const int& idFrame);

    void SetWorldPos(const cv::Mat& pos);
    cv::Mat GetWorldPos();

    cv::Mat GetNormal();
    KeyFrame* GetReferenceKeyFrame();

    std::map<KeyFrame*, size_t> GetObservations();
    int ObservationTimes();

    void AddObservation(KeyFrame* pKF, size_t idx);
    void EraseObservation(KeyFrame* pKF);

    int GetIndexInKeyFrame(KeyFrame* pKF);
    bool IsInKeyFrame(KeyFrame* pKF);

    void SetBadFlag();
    bool IsBad();

public:
    uint64_t id_;
    static uint64_t nextId_;
    int64_t firstKeyFrameId_;
    int64_t firstFrameId_;
    int32_t numObservations_;

    // 世界坐标系中的位置
    cv::Mat worldPos_;

    //
    std::map<KeyFrame*, size_t> observations_;

    // 参考关键帧
    KeyFrame* pRefKF_;

    bool badFlag_; // 指示这个点是否是好的

    Map* pMap_;

    std::mutex mutexPos_;
    std::mutex mutexFeatures_;
}



}
