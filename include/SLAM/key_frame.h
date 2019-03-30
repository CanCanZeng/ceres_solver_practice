
#include <set>
#include <map>
#include <mutex>

#include "SLAM/map_point.h"
#include "SLAM/frame.h"
#include "SLAM/key_frame_database.h"

namespace SLAM {

class Map;
class MapPoint;
class Frame;
class KeyFrameDatabase;

class KeyFrame
{
public:
    KeyFrame(Frame& frame, Map* pMap, KeyFrameDatabase* pKFDatabase);

    void SetPose(const cv::Mat& poseTcw);
    cv::Mat GetPose();
    cv::Mat GetPoseInverse();
    cv::Mat GetCameraCenter();
    cv::Mat GetRotation();
    cv::Mat GetTranslation();

    // 连通图相关的函数
    void AddConnection(KeyFrame* pKF, const int& weight);
    void EraseConnection(KeyFrame* pKF);
    void UpdateConnections();
    void UpdateBestCovisibles();
    std::set<KeyFrame*> GetConnectedKeyFrames();
    std::vector<KeyFrame*> GetVectorCovisibleKeyFrames();
    std::vector<KeyFrame*> GetBestCovisibleKeyFrames();
    std::vector<KeyFrame*> GetCovisiblesByWeight(const int& weight);
    int GetWeight(KeyFrame* pKF);

    // 最小生成树相关的函数
    void AddChild(KeyFrame* pKF);
    void EraseChild(KeyFrame* pKF);
    void ChangeParent(KeyFrame* pKF);
    std::set<KeyFrame*> GetChilds();
    KeyFrame* GetParent();
    bool HasChild(KeyFrame* pKF);

    // 回环 边
    void AddLoopEdge(KeyFrame* pKF);
    std::set<KeyFrame*> GetLoopEdges();

    // 地图点相关的函数
    void AddMapPoint(MapPoint* pMapPoint, const size_t& idx);
    void EraseMapPointMatch(const size_t& idx);
    void EraseMapPointMatch(MapPoint* pMapPoint);
    void ReplaceMapPointMatch(const size_t& idx, MapPoint* pMapPoint);
    std::set<MapPoint*> GetMapPoints();
    std::vector<MapPoint*> GetMapPointMatches();
    int TrackedMapPoints(const int& minObservations);
    MapPoint* GetMapPoint(const size_t& idx);

    bool IsInImage(const float&x, const float& y) const;

    // Enable/Disable bad flag changes
    void SetNotErase();
    void SetErase();

    void SetBadFlag();
    bool IsBad();

    static bool WeightCompare(int a, int b) {return a > b;}

    static bool KeyFrameIdCompare(KeyFrame* pKF1, KeyFrame* pKF2) {   return pKF1->id_ < pKF2->id_;   }


public:
    double timeStamp_;

    uint64_t id_;
    static uint64_t nextId_;
    const int64_t frameId_;

    const float fx_, fy_, cx_, cy_, invFx_, invFy_;
    const int32_t numOfKeyPoints;

    const std::vector<cv::KeyPoint> keyPoints_;
    const std::vector<cv::KeyPoint> keyPointsUndistorted_;
    const std::vector<float> depths_;

    // pose relative to parent (当 badflag 被设置的时候被计算）
    cv::Mat poseTcp_;

    // 图像边界
    const int32_t minX_, minY_, maxX_, maxY_;
    const cv::Mat K_;

    cv::Mat poseTcw_;
    cv::Mat poseTwc_;
    cv::Mat posetwc_; // 相机中心在世界坐标系中的坐标


    bool notErase_;
    bool tobeErased_;
    bool badFlag_;

    Map* pMap_;

    std::mutex mutexPose_;
    std::mutex mutexConnections_;
    std::mutex mutexFeatures_;

    // 局部地图用到的变量
    uint64_t numBALocalForKF;
    uint64_t numBAFixedForKF;

    //

};

}
