
#include <set>
#include <mutex>

#include "SLAM/map_point.h"
#include "SLAM/key_frame.h"

namespace SLAM
{

class MapPoint;
class KeyFrame;

class Map
{
public:
    void AddKeyFrame(KeyFrame* pKF);
    void EraseKeyFrame(KeyFrame* pKF);
    void AddMapPoint(MapPoint* pMP);
    void EraseMapPoint(MapPoint* pMP);

    void SetReferenceMapPoints(const std::vector<MapPoint*>& mapPoints);
    void InformNewBigChange();
    int32_t GetLastBigChangeIdx();

    std::vector<KeyFrame*> GetAllKeyFrames();
    std::vector<MapPoint*> GetAllMapPoints();
    std::vector<MapPoint*> GetReferenceMapPoints();

    uint64_t MapPointsInMap();
    uint64_t KeyFramesInMap();

    uint64_t GetMaxKFId();

    void Clear();

public:
    std::vector<KeyFrame*> keyFrameOrigins_;

    std::mutex mutexMapUpdate_;
    std::mutex mutexPointCreation_;
    std::mutex mutexMap_;

    std::set<MapPoint*> mapPoints_;
    std::set<KeyFrame*> keyFrames_;

    std::vector<MapPoint*> referenceMapPoints_;

    uint64_t maxKFId_;

    // Index related to a big change in the map (loop closure, global BA)
    int32_t bigChangeIdx_;
};



}


