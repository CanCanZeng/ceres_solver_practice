#ifndef SLAM_FRAME_H
#define SLAM_FRAME_H

#include <iostream>
#include <vector>

#include <Eigen/Core>

#include <opencv2/core.hpp>

#include "SLAM/map_point.h"

namespace SLAM
{

class Frame
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Frame() {}
    ~Frame();
    Eigen::Matrix3f Rwc()
    {
        return poseTwc_.block(0, 0, 3, 3);
    }

    Eigen::Vector3f twc()
    {
        return poseTwc_.block(0, 3, 3, 1);
    }

    void SetIntrinsicAndExtrinsic(const Eigen::Vector4f& intrinsic, const Eigen::Matrix4f& poseTwc);
    void ComputeProjectMatrix()
    {
        Eigen::Matrix3f K;
        K << intrinsic_[0], 0, intrinsic_[2], 0, intrinsic_[1], intrinsic_[3], 0, 0, 1;
        Eigen::Matrix3f R_wc = poseTwc_.block(0, 0, 3, 3);
        Eigen::Vector3f t_wc = poseTwc_.block(0, 3, 3, 1);
        projectionMatrix_.leftCols<3>() = R_wc.transpose();
        projectionMatrix_.rightCols<1>() = -R_wc.transpose() * t_wc;
        projectionMatrix_ = K * projectionMatrix_;
    }

public:
    cv::Mat img_;  // 图像
    size_t frameIndex_; // 图像的索引
    std::vector<cv::KeyPoint> keypoints_;  // 关键点
    std::vector<MapPoint*> pMapPoints_;    // 关键点对应的地图点
    cv::Mat descriptors_;                   // 关键点对应的描述子

    Eigen::Matrix4f poseTwc_;  // 位姿
    Eigen::Vector4f intrinsic_; // 内参
    Eigen::Matrix<float, 3, 4> projectionMatrix_;  // 投影矩阵，用来做三角化的
};

Frame::~Frame()
{
//    for(size_t i=0, iEnd=pMapPoints_.size(); i<iEnd; ++i)
//    {
//        if(pMapPoints_[i] != nullptr)
//        {
//            delete pMapPoints_[i];
//            pMapPoints_[i] = nullptr;
//        }
//    }
}

}

#endif // SLAM_FRAME_H
