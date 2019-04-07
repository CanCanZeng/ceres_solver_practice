#ifndef SLAM_MAP_POINT_H
#define SLAM_MAP_POINT_H

#include <vector>

#include <Eigen/Core>

namespace SLAM
{

class Frame;
class Map;

struct Observation
{
public:
    Observation(Frame* pFrame, const size_t& keypointIdx)
        :pFrame_(pFrame), keypointIdx_(keypointIdx)
    {}

public:

    size_t keypointIdx_;
    Frame* pFrame_;
//    MapPoint* pMapPoint_;
};

class MapPoint
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ~MapPoint();

    // 生成一个已经三角化过的地图点
    MapPoint(const Eigen::Vector3f& position, Frame* pRefFrame)
        :position_(position),  pRefFrame_(pRefFrame), badFlag_(false) {}

    // 生成一个还没有被三角化的地图点
    MapPoint(Frame* pRefFrame)
        :position_(Eigen::Vector3f(0, 0, 0)), pRefFrame_(pRefFrame), badFlag_(false) {}

    void AddObservation(Observation* observation) {observations_.push_back(observation);}

public:

    Eigen::Vector3f position_;
    std::vector<Observation*> observations_;

    // 参考关键帧
    Frame* pRefFrame_;
    bool badFlag_; // 指示这个点是否是好的

    Map* pMap_;
};

MapPoint::~MapPoint()
{
    for(auto pObserv : observations_)
    {
        if(pObserv != nullptr)
        {
            delete pObserv;
            pObserv = nullptr;
        }
    }
    observations_.clear();
}

}

#endif // SLAM_MAP_POINT_H
