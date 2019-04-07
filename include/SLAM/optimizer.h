#ifndef SLAM_OPTIMIZER_H
#define SLAM_OPTIMIZER_H

#include <iostream>
#include <vector>

#include <Eigen/Core>

#include <ceres/ceres.h>

#include "SLAM/frame.h"
#include "SLAM/map.h"

namespace SLAM
{

class Optimizer
{
public:
    ~Optimizer()
    {
        if(cameraParameters_ != nullptr)
            delete[] cameraParameters_;
        if(pts3d_ != nullptr)
            delete[] pts3d_;
    }

    void BuildOptimizationProblem(Map& currMap);
    void SolveProblem();
    void GetResult(Map& currMap);

    void CopyCameraParams(double* pCameraParams, Frame* frame); // 从frame复制到pCameraParams中
    void CopyCameraParams(Frame* frame, double* pCameraParams); // 从 pCameraParams复制到frame中
    void CopyMapPointParams(double* pPt3d, MapPoint* mapPoint); // mapPoint -> pPt3d
    void CopyMapPointParams(MapPoint* mapPoint, double* pPt3d); // pPt3d -> mapPoint

public:
    ceres::Problem problem_;

    uint32_t numOfMapPoints_;
    uint32_t numOfCameras_;
    uint32_t numOfObservations_;
    double* cameraParameters_ = nullptr;
    double* pts3d_ = nullptr;
    std::map<uint32_t, MapPoint*> mapFromPointIndex2MapPoint_;
    std::map<Frame*, uint32_t> mapFromFrame2CameraIndex_;
};

void Optimizer::BuildOptimizationProblem(Map& currMap)
{
    uint32_t numOfMapPoints = currMap.data_.size();
    uint32_t numOfCameras = currMap.pFrameDatabase_->frames_.size();
    uint32_t numOfObservations = 0;
    for(auto pMapPoint: currMap.data_)
    {
        numOfObservations += pMapPoint->observations_.size();
    }

    numOfCameras_ = numOfCameras;
    numOfMapPoints_ = numOfMapPoints;
    numOfObservations_ = numOfObservations;
    std::cout << "number of cameras: " << numOfCameras << std::endl;
    std::cout << "number of map points: " << numOfMapPoints << std::endl;
    std::cout << "number of observations: " << numOfObservations << std::endl;

    cameraParameters_ = new double[numOfCameras*6];
    pts3d_ = new double[numOfMapPoints*3];

    const std::vector<Frame*>& frames = currMap.pFrameDatabase_->frames_;
    for(uint32_t i=0; i<numOfCameras; ++i)
    {
        double* pCameraParams = cameraParameters_ + 6*i;
        CopyCameraParams(pCameraParams, frames[i]);
        mapFromFrame2CameraIndex_[frames[i]] = i;
    }

    uint32_t index = 0;
    for(auto pMapPoint : currMap.data_)
    {
        double* pPts3d = pts3d_ + 3*index;
        CopyMapPointParams(pPts3d, pMapPoint);
        mapFromPointIndex2MapPoint_[index] = pMapPoint;

        for(auto pObservation : pMapPoint->observations_)
        {
            Frame* pFrame = pObservation->pFrame_;
            cv::KeyPoint kpt = pFrame->keypoints_[pObservation->keypointIdx_];
            double fx = pFrame->intrinsic_[0];
            double fy = pFrame->intrinsic_[1];
            double cx = pFrame->intrinsic_[2];
            double cy = pFrame->intrinsic_[3];

            uint32_t cameraIndex = mapFromFrame2CameraIndex_[pFrame];
            double* pCameraParam = cameraParameters_ + cameraIndex * 6;

            ceres::CostFunction* cost_function =
                ReprojectionErrorWithAngleAxis::Create(kpt.pt.x, kpt.pt.y, fx, fy, cx, cy);
            problem_.AddResidualBlock(cost_function,
                                     new ceres::HuberLoss(2) /* squared loss */,
//                                      NULL,
                                     pCameraParam,
                                     pPts3d);
        }
        index++;
    }
}

void Optimizer::SolveProblem()
{
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = 100;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem_, &summary);
    std::cout << summary.FullReport() << "\n";
}

void Optimizer::GetResult(Map &currMap)
{
    for(size_t i=0; i<numOfMapPoints_; ++i)
    {
        MapPoint* pMapPoint = mapFromPointIndex2MapPoint_[i];
        double* pPt3d = pts3d_ + 3*i;
        CopyMapPointParams(pMapPoint, pPt3d);
    }

    for(size_t i=0; i<numOfCameras_; ++i)
    {
        Frame* frame = currMap.pFrameDatabase_->frames_[i];
        double* pCameraParams = cameraParameters_ + 6*i;
        CopyCameraParams(frame, pCameraParams);
    }
}

void Optimizer::CopyCameraParams(double *pCameraParams, Frame *frame)
{
    Eigen::Matrix3f Rwc = frame->Rwc();
    Eigen::Vector3f twc = frame->twc();

    Eigen::Matrix3f Rcw = Rwc.transpose();
    Eigen::Vector3f tcw = -Rwc.transpose() * twc;

    Eigen::AngleAxisf q(Rcw);
    Eigen::Vector3f temp = q.axis() * q.angle();
    pCameraParams[0] = temp[0];
    pCameraParams[1] = temp[1];
    pCameraParams[2] = temp[2];

    pCameraParams[3] = tcw[0];
    pCameraParams[4] = tcw[1];
    pCameraParams[5] = tcw[2];
}

void Optimizer::CopyCameraParams(Frame *frame, double *pCameraParams)
{
    Eigen::Vector3f tcw(pCameraParams[3], pCameraParams[4], pCameraParams[5]);


    Eigen::Vector3f axis(pCameraParams[0], pCameraParams[1], pCameraParams[2]);
    float angle = axis.norm();
    axis /= angle;

    Eigen::AngleAxisf r(angle, axis);
    Eigen::Matrix3f Rcw(r);

    Eigen::Matrix3f Rwc = Rcw.transpose();
    Eigen::Vector3f twc = -Rcw.transpose() * tcw;

    frame->poseTwc_.block(0, 0, 3, 3) = Rwc;
    frame->poseTwc_.block(0, 3, 3, 1) = twc;
}

void Optimizer::CopyMapPointParams(double *pPt3d, MapPoint *mapPoint)
{
    Eigen::Vector3f position = mapPoint->position_;
    pPt3d[0] = position[0];
    pPt3d[1] = position[1];
    pPt3d[2] = position[2];
}

void Optimizer::CopyMapPointParams(MapPoint *mapPoint, double *pPt3d)
{
    Eigen::Vector3f& position = mapPoint->position_;
    position[0] = pPt3d[0];
    position[1] = pPt3d[1];
    position[2] = pPt3d[2];
}

} // namespace SLAM

#endif // SLAM_OPTIMIZER_H
