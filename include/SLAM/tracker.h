#ifndef SLAM_TRACTER_H
#define SLAM_TRACTER_H

#include <list>
#include <mutex>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>

#include "SLAM/map.h"
#include "SLAM/frame.h"
#include "SLAM/frame_database.h"

namespace SLAM
{

class Tracker
{
public:
    Tracker();

    int TrackImageMonocular(cv::Mat img, const Eigen::Matrix3f& Rwc, const Eigen::Vector3f& twc, const Eigen::Vector4f& intrin);
    void CreateOneFrame(Frame* pFrame, cv::Mat img, const Eigen::Matrix3f& Rwc, const Eigen::Vector3f& twc, const Eigen::Vector4f& intrin);
    void MatchFrameDataset(Frame* pFrame, FrameDatabase* pFrameDatabase);
    // 结束的时候，做一个环，用最开始的几帧跟最后几帧进行匹配
    void CreateLoop();

    static void FindGoodMatches(std::vector<uchar>& isGoodMatch, const std::vector<std::vector<cv::DMatch>>& knnMatches, const float& ratioThresh);
    static bool TriangulateMapPoint(MapPoint* pMapPoint, const cv::DMatch& match, Frame* frame1, Frame* frame2);
    static bool TriangulateMapPoint(Eigen::Vector3f& position3d, const cv::Point2f& kp1, const cv::Point2f& kp2, const Eigen::Matrix<float, 3, 4>& projMat1, const Eigen::Matrix<float, 3, 4>& projMat2);

public:
    cv::Mat currImg_;
    cv::Mat currFrame_;
    size_t nextFrameID_ = 0;
    cv::Ptr<cv::AKAZE> featureDetector_;
    cv::BFMatcher matcher_;

    const size_t N_ = 3; // 每一次跟前面的几帧进行匹配

    FrameDatabase* pFrameDatabase_;
    Map* pMap_;
};

Tracker::Tracker()
{
    featureDetector_ = cv::AKAZE::create();
}

int Tracker::TrackImageMonocular(cv::Mat img, const Eigen::Matrix3f &Rwc, const Eigen::Vector3f &twc, const Eigen::Vector4f &intrin)
{
    // 插入一帧
    Frame* pFrame(new Frame);
    CreateOneFrame(pFrame, img, Rwc, twc, intrin);
    MatchFrameDataset(pFrame, pFrameDatabase_);
    pFrameDatabase_->AddFrame(pFrame);

    return 0;
}

void Tracker::CreateOneFrame(Frame *pFrame, cv::Mat img, const Eigen::Matrix3f &Rwc, const Eigen::Vector3f &twc, const Eigen::Vector4f &intrin)
{
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    featureDetector_->detectAndCompute(img, cv::Mat(), keypoints, descriptors);

    Eigen::Matrix4f poseTwc;
    poseTwc.block(0, 0, 3, 3) = Rwc;
    poseTwc.block(0, 3, 3, 1) = twc;

    pFrame->frameIndex_ = nextFrameID_++;
    pFrame->img_ = img;
    pFrame->keypoints_ = keypoints;
    pFrame->pMapPoints_.resize(keypoints.size(), nullptr);
    pFrame->descriptors_ = descriptors;
    pFrame->intrinsic_ = intrin;
    pFrame->poseTwc_ = poseTwc;
    pFrame->ComputeProjectMatrix();
}

// 最后一个参数表征 逆向匹配， 也就是在最后结束的时候，最开始的几帧跟最后面几帧进行匹配
void Tracker::MatchFrameDataset(Frame *pFrame, FrameDatabase *pFrameDatabase)
{
    size_t sizeOfFrameDataset = pFrameDatabase->frames_.size();
    std::vector<uchar> isMatchGood;

    size_t iStart, iEnd;
    iStart = sizeOfFrameDataset >= N_ ? sizeOfFrameDataset - N_ : 0;
    iEnd = sizeOfFrameDataset;
    for(size_t i=iStart; i<iEnd; ++i)
    {
        Frame* queryFrame = pFrame;
        Frame* trainFrame = pFrameDatabase->frames_[i];
        std::vector<std::vector<cv::DMatch>> knnMatches;
        matcher_.knnMatch(queryFrame->descriptors_, trainFrame->descriptors_, knnMatches, 2);
        Tracker::FindGoodMatches(isMatchGood, knnMatches, 0.7f);

        std::vector<cv::DMatch> goodMatches;
        for(size_t iii=0; iii<isMatchGood.size(); ++iii)
        {
            if(isMatchGood[iii] != 0)
                goodMatches.push_back(knnMatches[iii][0]);
        }
//            cv::Mat matchImg2;
//            cv::drawMatches(frame_i->img_, frame_i->keypoints_, frame_j->img_, frame_j->keypoints_,
//                            goodMatches, matchImg2);
//            cv::imshow("matches2", matchImg2);
//            cv::waitKey(30);

        for(int k=0, kEnd=goodMatches.size(); k<kEnd; ++k)
        {

            // 看 query 点是否已经是某个mappoint的观测了，如果不是就应该生成一个新的mappoint
            const cv::DMatch& match = goodMatches[k];
            size_t queryPointIdx = match.queryIdx;
            size_t trainPointIdx = match.trainIdx;
            if(queryFrame->pMapPoints_[queryPointIdx] != NULL)
            {
                // 将地图点投影回来，如果重投影误差太大，说明可能是误匹配，不应该添加这次观测
                MapPoint* pMapPoint = queryFrame->pMapPoints_[queryPointIdx];
                Eigen::Vector3f& pt3dWorld = pMapPoint->position_;
                Eigen::Vector4f pt4dWorld(pt3dWorld[0], pt3dWorld[1], pt3dWorld[2], 1);
                Eigen::Vector3f projectedPoint = trainFrame->projectionMatrix_ * pt4dWorld;
                float x = projectedPoint[0] / projectedPoint[2];
                float y = projectedPoint[1] / projectedPoint[2];

                cv::Point2f kp = trainFrame->keypoints_[trainPointIdx].pt;
                float deltX = x - kp.x;
                float deltY = y - kp.y;

                if(std::fabs(deltX) > 10 || std::fabs(deltY) > 10)
                    continue;

                // 地图点增加了一次观测
                Observation* observation(new Observation(trainFrame, trainPointIdx));
                queryFrame->pMapPoints_[queryPointIdx]->AddObservation(observation);

                // trainFrame 也有多了一个地图点
                trainFrame->pMapPoints_[trainPointIdx] = queryFrame->pMapPoints_[queryPointIdx];
            }
            else
            {
                // 新增一个地图点
                MapPoint* pMapPoint(new MapPoint(queryFrame));

                // 对这个地图点进行三角化
                bool triangulateSuccess = Tracker::TriangulateMapPoint(pMapPoint, match, queryFrame, trainFrame);
                if(!triangulateSuccess)
                    continue;

                // queryFrame 和 trainFrame 都增加一个地图点
                queryFrame->pMapPoints_[queryPointIdx] = pMapPoint;
                trainFrame->pMapPoints_[trainPointIdx] = pMapPoint;

                // 该地图点增加两次观测
                Observation* obs1(new Observation(trainFrame, trainPointIdx));
                Observation* obs2(new Observation(queryFrame, queryPointIdx));
                pMapPoint->AddObservation(obs1);
                pMapPoint->AddObservation(obs2);

                // 将地图点添加到地图中
                pMap_->AddMapPoint(pMapPoint);
            }
        }
    }
}

void Tracker::CreateLoop()
{
    size_t sizeOfFrameDataset = pFrameDatabase_->frames_.size();
    std::vector<uchar> isMatchGood;

    for(size_t i=0, iEnd=N_; i<iEnd; ++i)
    {
        Frame* queryFrame = pFrameDatabase_->frames_[i];

        size_t jStart = sizeOfFrameDataset + i - N_ - 1;
        size_t jEnd = sizeOfFrameDataset - 1;
        for(size_t j=jStart; j<jEnd; ++j)
        {
            Frame* trainFrame = pFrameDatabase_->frames_[j];

            std::vector<std::vector<cv::DMatch>> knnMatches;
            matcher_.knnMatch(queryFrame->descriptors_, trainFrame->descriptors_, knnMatches, 2);
            Tracker::FindGoodMatches(isMatchGood, knnMatches, 0.7f);

            std::vector<cv::DMatch> goodMatches;
            for(size_t iii=0; iii<isMatchGood.size(); ++iii)
            {
                if(isMatchGood[iii] != 0)
                    goodMatches.push_back(knnMatches[iii][0]);
            }
    //            cv::Mat matchImg2;
    //            cv::drawMatches(frame_i->img_, frame_i->keypoints_, frame_j->img_, frame_j->keypoints_,
    //                            goodMatches, matchImg2);
    //            cv::imshow("matches2", matchImg2);
    //            cv::waitKey(30);

            for(int k=0, kEnd=goodMatches.size(); k<kEnd; ++k)
            {

                // 看 query 点是否已经是某个mappoint的观测了，如果不是就应该生成一个新的mappoint
                const cv::DMatch& match = goodMatches[k];
                size_t queryPointIdx = match.queryIdx;
                size_t trainPointIdx = match.trainIdx;
                std::vector<MapPoint*>& queryMapPoints = queryFrame->pMapPoints_;
                std::vector<MapPoint*>& trainMapPoints = trainFrame->pMapPoints_;
                if(queryMapPoints[queryPointIdx] != nullptr && trainMapPoints[trainPointIdx] == nullptr)
                {
                    // 将地图点投影回来，如果重投影误差太大，说明可能是误匹配，不应该添加这次观测
                    MapPoint* pMapPoint = queryMapPoints[queryPointIdx];
                    Eigen::Vector3f& pt3dWorld = pMapPoint->position_;
                    Eigen::Vector4f pt4dWorld(pt3dWorld[0], pt3dWorld[1], pt3dWorld[2], 1);
                    Eigen::Vector3f projectedPoint = trainFrame->projectionMatrix_ * pt4dWorld;
                    float x = projectedPoint[0] / projectedPoint[2];
                    float y = projectedPoint[1] / projectedPoint[2];

                    cv::Point2f kp = trainFrame->keypoints_[trainPointIdx].pt;
                    float deltX = x - kp.x;
                    float deltY = y - kp.y;

                    if(std::fabs(deltX) > 10 || std::fabs(deltY) > 10)
                        continue;

                    // 地图点增加了一次观测
                    Observation* observation(new Observation(trainFrame, trainPointIdx));
                    queryMapPoints[queryPointIdx]->AddObservation(observation);

                    // trainFrame 也有多了一个地图点
                    trainMapPoints[trainPointIdx] = queryMapPoints[queryPointIdx];
                }
                if(queryMapPoints[queryPointIdx] == nullptr && trainMapPoints[trainPointIdx] != nullptr)
                {
                    // 将地图点投影回来，如果重投影误差太大，说明可能是误匹配，不应该添加这次观测
                    MapPoint* pMapPoint = trainMapPoints[trainPointIdx];
                    Eigen::Vector3f& pt3dWorld = pMapPoint->position_;
                    Eigen::Vector4f pt4dWorld(pt3dWorld[0], pt3dWorld[1], pt3dWorld[2], 1);
                    Eigen::Vector3f projectedPoint = queryFrame->projectionMatrix_ * pt4dWorld;
                    float x = projectedPoint[0] / projectedPoint[2];
                    float y = projectedPoint[1] / projectedPoint[2];

                    cv::Point2f kp = queryFrame->keypoints_[queryPointIdx].pt;
                    float deltX = x - kp.x;
                    float deltY = y - kp.y;

                    if(std::fabs(deltX) > 10 || std::fabs(deltY) > 10)
                        continue;

                    // 地图点增加了一次观测
                    Observation* observation(new Observation(queryFrame, queryPointIdx));
                    trainMapPoints[trainPointIdx]->AddObservation(observation);

                    // queryFrame 也有多了一个地图点
                    queryMapPoints[queryPointIdx] = trainMapPoints[trainPointIdx];
                }
                // 如果两者都观测到了，有点尴尬了
                if(queryMapPoints[queryPointIdx] != nullptr && trainMapPoints[trainPointIdx] != nullptr)
                {
                    // 两者是同一个地图点，相安无事
                    if(queryMapPoints[queryPointIdx] == trainMapPoints[trainPointIdx])
                        continue;
                    else
                    {
                        // 将地图点投影回来，如果重投影误差太大，说明可能是误匹配，不应该添加这次观测
                        MapPoint* pMapPoint = queryMapPoints[queryPointIdx];
                        Eigen::Vector3f& pt3dWorld = pMapPoint->position_;
                        Eigen::Vector4f pt4dWorld(pt3dWorld[0], pt3dWorld[1], pt3dWorld[2], 1);
                        Eigen::Vector3f projectedPoint = trainFrame->projectionMatrix_ * pt4dWorld;
                        float x = projectedPoint[0] / projectedPoint[2];
                        float y = projectedPoint[1] / projectedPoint[2];

                        cv::Point2f kp = trainFrame->keypoints_[trainPointIdx].pt;
                        float deltX = x - kp.x;
                        float deltY = y - kp.y;

                        // 重投影误差太大，可能是误匹配，跳过，否则进行合并
                        if(std::fabs(deltX) > 2 || std::fabs(deltY) > 2)
                            continue;
                        MapPoint* pMapPoint2 = trainMapPoints[trainPointIdx];
                        // 将会留下来的点
                        MapPoint* pMapPointFinale = pMapPoint->observations_.size() > pMapPoint2->observations_.size() ? pMapPoint : pMapPoint2;
                        // 将会被删除的点
                        MapPoint* pMapPointTobeDeleted = pMapPoint->observations_.size() > pMapPoint2->observations_.size() ? pMapPoint2 : pMapPoint;
                        // 将观测进行转移
                        for(auto observ : pMapPointTobeDeleted->observations_)
                        {
                            pMapPointFinale->observations_.push_back(observ);  // 为这个点增加一次观测
                            observ->pFrame_->pMapPoints_[observ->keypointIdx_] = pMapPointFinale; // 将所有之前的观测都指向新的地图点
                        }
                        delete pMapPointTobeDeleted;
                    }
                }
            }
        }
    }
}


void Tracker::FindGoodMatches(std::vector<uchar> &isGoodMatch, const std::vector<std::vector<cv::DMatch> > &knnMatches, const float &ratioThresh)
{
    size_t numOfMatches = knnMatches.size();
    isGoodMatch.clear();
    isGoodMatch.resize(numOfMatches, 0);

    for(size_t i=0; i<numOfMatches; ++i)
    {
        if(knnMatches[i][0].distance < ratioThresh*knnMatches[i][1].distance)
            isGoodMatch[i] = 255;
    }
}

bool Tracker::TriangulateMapPoint(MapPoint *pMapPoint, const cv::DMatch &match, Frame *frame1, Frame *frame2)
{
    return Tracker::TriangulateMapPoint(pMapPoint->position_, frame1->keypoints_[match.queryIdx].pt, frame2->keypoints_[match.trainIdx].pt,
            frame1->projectionMatrix_, frame2->projectionMatrix_);
}

bool Tracker::TriangulateMapPoint(Eigen::Vector3f& position3d, const cv::Point2f& kp1, const cv::Point2f& kp2, const Eigen::Matrix<float, 3, 4>& projMat1, const Eigen::Matrix<float, 3, 4>& projMat2)
{
    int num_of_pose = 2;
    Eigen::MatrixXf svd_A(2 * num_of_pose, 4);

    Eigen::Matrix<float, 3, 4> P;
    P = projMat1;
    svd_A.row(0) = kp1.x * P.row(2) - P.row(0);
    svd_A.row(1) = kp1.y * P.row(2) - P.row(1);
    P = projMat2;
    svd_A.row(2) = kp2.x * P.row(2) - P.row(0);
    svd_A.row(3) = kp2.y * P.row(2) - P.row(1);

    Eigen::Vector4f svd_V = Eigen::JacobiSVD<Eigen::MatrixXf>(svd_A, Eigen::ComputeThinV).matrixV().rightCols<1>();

    position3d[0] = svd_V[0] / svd_V[3];
    position3d[1] = svd_V[1] / svd_V[3];
    position3d[2] = svd_V[2] / svd_V[3];

    // 验证一下结果
    Eigen::Vector4f position4d(position3d[0], position3d[1], position3d[2], 1);
    Eigen::Vector3f projectedPoint = projMat1 * position4d;
    float x = projectedPoint[0] / projectedPoint[2];
    float y = projectedPoint[1] / projectedPoint[2];
    float deltX = x - kp1.x;
    float deltY = y - kp1.y;

    if(std::fabs(deltX) > 1 || std::fabs(deltY) > 1)
        return false;

    projectedPoint = projMat2 * position4d;
    x = projectedPoint[0] / projectedPoint[2];
    y = projectedPoint[1] / projectedPoint[2];
    deltX = x - kp2.x;
    deltY = y - kp2.y;

    if(std::fabs(deltX) > 1 || std::fabs(deltY) > 1)
        return false;

    return true;  // 三角化成功了
}


}

#endif // SLAM_TRACTER_H
