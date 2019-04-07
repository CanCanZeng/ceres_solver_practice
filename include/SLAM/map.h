#ifndef SLAM_MAP_H
#define SLAM_MAP_H

#include <set>
#include <mutex>

#include "SLAM/map_point.h"

namespace SLAM
{

class MapPoint;
class KeyFrame;
class FrameDatabase;

class Map
{
public:
    ~Map();

    size_t size() {return data_.size();}
    void AddMapPoint(MapPoint* pMapPoint)
    {
        data_.insert(pMapPoint);
    }

    void DeleteMapPoint(MapPoint* pMapPoint)
    {
        auto result = data_.find(pMapPoint);
        if(result != data_.end())
            data_.erase(pMapPoint);
    }

    void ArrageMapPoints();

    bool SaveMapPoints(std::string plyName);
    static bool SavePointCloud(std::string plyName, const std::vector<Eigen::Vector3f>& pointCloud);

public:
    std::set<MapPoint*> data_;
    FrameDatabase* pFrameDatabase_;
};


Map::~Map()
{
    for(auto iter=data_.begin(); iter!=data_.end(); ++iter)
    {
        MapPoint* pMapPoint = *iter;
        if(pMapPoint != nullptr)
        {
            data_.erase(pMapPoint);
        }
    }
}

void Map::ArrageMapPoints()
{
    // 遍历一遍地图点，如果观测次数少于3次，直接剔除
    for(auto iter = data_.begin(); iter != data_.end(); ++iter)
    {
        auto pMapPoint = *iter;
        if(pMapPoint->observations_.size() < 3)
        {
            DeleteMapPoint(pMapPoint);
        }
    }
}

bool Map::SaveMapPoints(std::string plyName)
{
    // 将所有的地图点保存下来看看
    std::vector<Eigen::Vector3f> pointCloud;
    for(auto pMapPoint : data_)
    {
        Eigen::Vector3f& pt = pMapPoint->position_;
        pointCloud.push_back(pt);
    }
    return Map::SavePointCloud(plyName, pointCloud);
}

bool Map::SavePointCloud(std::string plyName, const std::vector<Eigen::Vector3f> &pointCloud)
{
    std::ofstream stream(plyName.c_str());
    if (!stream)
    {
        return false;
    }

    size_t numPoints = pointCloud.size();
    stream << "ply" << std::endl;
    stream << "format ascii 1.0" << std::endl;
    stream << "element vertex " << numPoints << std::endl;
    stream << "property float x" << std::endl;
    stream << "property float y" << std::endl;
    stream << "property float z" << std::endl;
    stream << "end_header" << std::endl;

    for (const Eigen::Vector3f& vert : pointCloud)
    {
        stream << vert[0] << " " << vert[1] << " " << vert[2];

        stream << std::endl;
    }
    std::cout << "saved local mesh!" << std::endl;

    return true;
}

}

#endif  // SLAM_MAP_H
