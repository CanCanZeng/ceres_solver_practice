#include <iostream>
#include <fstream>
#include <map>
#include <set>
#include <memory>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "helper_functions.h"
#include "image_source_manager.h"

class Observation;
class MapPoint;
class Map;
class Frame;
class FrameDataset;

class Observation
{
public:
    Observation(Frame* pFrame, const size_t& keypointIdx)
        :pFrame_(pFrame), keypointIdx_(keypointIdx)
    {}

public:

    size_t keypointIdx_;
    Frame* pFrame_;
    MapPoint* pMapPoint_;
};

class MapPoint
{
public:
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

class Map
{
public:
    size_t size() {return data.size();}
    void AddMapPoint(MapPoint* pMapPoint)
    {
        data.insert(pMapPoint);
    }

public:
    std::set<MapPoint*> data;
};

class Frame
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Frame() {}
    Eigen::Matrix3f Rwc()
    {
        return poseTwc_.block(0, 0, 3, 3);
    }

    Eigen::Vector3f twc()
    {
        return poseTwc_.block(0, 3, 3, 1);
    }

    void SetIntrinsicAndExtrinsic(const Eigen::Vector4f& intrinsic, const Eigen::Matrix4f& poseTwc);

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


class FrameDataset
{
public:
    void AddFrame(Frame* frame) { frames_.push_back(frame);}
//    void EraseFrame(Frame* frame);
    inline size_t size() {return frames_.size();}

    Frame* operator[](size_t i) {return frames_[i];}

    std::vector<Frame*> frames_;
};



//void FindGoodMatches(std::vector<cv::DMatch>& GoodMatchePoints, const cv::flann::Index& flannIndex, const cv::Mat& imageDesc2)
//{
//    cv::Mat macthIndex(imageDesc2.rows, 2, CV_32SC1);  // 最近的两个匹配的索引
//    cv::Mat matchDistance(imageDesc2.rows, 2, CV_32FC1);  // 最近的两个匹配对应的距离
//    flannIndex.knnSearch(imageDesc2, macthIndex, matchDistance, 2, cv::flann::SearchParams());

//    // Lowe's algorithm,获取优秀匹配点
//    for (int i = 0; i < matchDistance.rows; i++)
//    {
//        if (matchDistance.at<float>(i,0) < 0.6 * matchDistance.at<float>(i, 1))
//        {
//            cv::DMatch dmatches(i, macthIndex.at<int>(i, 0), matchDistance.at<float>(i, 0));
//            GoodMatchePoints.push_back(dmatches);
//        }
//    }
//}

void FindGoodMatches(std::vector<uchar>& isGoodMatch, const std::vector<std::vector<cv::DMatch>>& knnMatches, const float& ratioThresh)
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

//void FilterOutlierByFMatrix(std::vector<bool>& isMatcheGood, const std::vector<cv::DMatch>& matches,
//                            const std::vector<cv::KeyPoint>& keypoints1, const std::vector<cv::KeyPoint>& keypoints2,
//                            Eigen::Matrix4f& poseTwc1, const Eigen::Matrix4f& poseTwc2, float fx, float fy, float cx, float cy)
//{
//    // ---------------- 计算基础矩阵 F ----------------------
//    Eigen::Matrix3f F; // 基础矩阵
//    // 把分量提取出来
//    Eigen::Matrix3f R1 = poseTwc1.block(0, 0, 3, 3);
//    Eigen::Matrix3f R2 = poseTwc2.block(0, 0, 3, 3);
//    Eigen::Vector3f t1 = poseTwc1.block(0, 3, 3, 1);
//    Eigen::Vector3f t2 = poseTwc2.block(0, 3, 3, 1);
//    // 计算相对 pose
//    Eigen::Matrix3f R21 = R2.transpose() * R1;
//    Eigen::Vector3f t21 = R2.transpose() * (t1 - t2);
//    Eigen::Matrix3f t21_hat; // 用于叉乘
//    t21_hat <<
//            0, -t21[2], t21[1],
//            t21[2], 0, -t21[0],
//            -t21[1], t21[0], 0;

//    Eigen::Matrix3f K; // 内参矩阵
//    K << fx, 0, cx, 0, fy, cy, 0, 0, 1;

//    Eigen::Matrix3f K_inv = K.inverse();
//    F = K_inv.transpose() * t21_hat * R21 * K_inv;

//    // extract fundamental matrix
//    float f00 = F(0, 0); float f01 = F(0, 1); float f02 = F(0, 2);
//    float f10 = F(1, 0); float f11 = F(1, 1); float f12 = F(1, 2);
//    float f20 = F(2, 0); float f21 = F(2, 1); float f22 = F(2, 2);

//    // loop variables
//    float u1,v1,u2,v2;
//    float x2tFx1;
//    float Fx1u,Fx1v,Fx1w;
//    float Ftx2u,Ftx2v;

//    isMatcheGood.resize(matches.size(), false);
//    float inlier_threshold = 1.0f;  // 在640*480尺寸的时候，设置阈值为1是比较好的，那么图片如果缩小了，应该也要把阈值减小一点比较好
//    for(int i=0, iEnd=matches.size(); i<iEnd; ++i)
//    {
//        size_t queryIdx = matches[i].queryIdx;
//        size_t trainIdx = matches[i].trainIdx;
//        u1 = keypoints1[queryIdx].pt.y;
//        v1 = keypoints1[queryIdx].pt.x;
//        u2 = keypoints2[trainIdx].pt.y;
//        v2 = keypoints2[trainIdx].pt.x;

//        // F*x1
//        Fx1u = f00*u1+f01*v1+f02;
//        Fx1v = f10*u1+f11*v1+f12;
//        Fx1w = f20*u1+f21*v1+f22;

//        // F'*x2
//        Ftx2u = f00*u2+f10*v2+f20;
//        Ftx2v = f01*u2+f11*v2+f21;

//        // x2'*F*x1
//        x2tFx1 = u2*Fx1u+v2*Fx1v+Fx1w;

//        // sampson distance
//        float d = x2tFx1*x2tFx1 / (Fx1u*Fx1u+Fx1v*Fx1v+Ftx2u*Ftx2u+Ftx2v*Ftx2v);

//        if(fabs(d)<inlier_threshold)
//        {
//            isMatcheGood[i] = true;
//        }
//        else
//            isMatcheGood[i] = false;

//    }
//}

//void FilterOutlierByFMatrix(std::vector<bool>& isMatchGood, const std::vector<cv::DMatch>& matches,
//                            Frame* frame1, Frame* frame2)
//{
//    FilterOutlierByFMatrix(isMatchGood, matches, frame1->keypoints_, frame2->keypoints_, frame1->poseTwc_, frame2->poseTwc_,
//                           frame1->intrinsic_[0], frame1->intrinsic_[1], frame1->intrinsic_[2], frame1->intrinsic_[3]);
//}

cv::Mat FilterOutlierByFMatrix(std::vector<uchar>& isMatcheGood, const std::vector<cv::DMatch>& matches,
                            const std::vector<cv::KeyPoint>& keypoints1, const std::vector<cv::KeyPoint>& keypoints2)
{
    vector<cv::Point2f> srcPoints(matches.size());
    vector<cv::Point2f> dstPoints(matches.size());

    for (size_t i = 0; i < matches.size(); i++) {
        srcPoints[i] = keypoints1[matches[i].trainIdx].pt;
        dstPoints[i] = keypoints2[matches[i].queryIdx].pt;
    }

    //find homography matrix and get inliers mask
    isMatcheGood.resize(matches.size());
    cv::Mat fundamentalMatrix = cv::findHomography(srcPoints, dstPoints, isMatcheGood);
    return fundamentalMatrix;
}

cv::Mat FilterOutlierByFMatrix(std::vector<uchar>& isMatchGood, const std::vector<cv::DMatch>& matches,
                            Frame* frame1, Frame* frame2)
{
    FilterOutlierByFMatrix(isMatchGood, matches, frame1->keypoints_, frame2->keypoints_);
}

void TriangulateMapPoint(Eigen::Vector3f& position3d, const cv::Point2f& kp1, const cv::Point2f& kp2, const Eigen::Matrix<float, 3, 4>& projMat1, const Eigen::Matrix<float, 3, 4>& projMat2)
{
    int num_of_pose = 2;
    Eigen::MatrixXf svd_A(2 * num_of_pose, 4);

    Eigen::Matrix<float, 3, 4> P;
    P = projMat1;
    svd_A.row(0) = kp1.x * P.row(2) - P.row(0);
    svd_A.row(1) = kp1.y * P.row(2) - P.row(1);
    P = projMat2;
    svd_A.row(3) = kp2.x * P.row(2) - P.row(0);
    svd_A.row(4) = kp2.y * P.row(2) - P.row(1);

    Eigen::Vector4f svd_V = Eigen::JacobiSVD<Eigen::MatrixXf>(svd_A, Eigen::ComputeThinV).matrixV().rightCols<1>();

   position3d[0] = svd_V[0] / svd_V[3];
   position3d[1] = svd_V[1] / svd_V[3];
   position3d[2] = svd_V[2] / svd_V[3];
}

void TriangulateMapPoint(MapPoint* pMapPoint, const cv::DMatch& match, Frame* frame1, Frame* frame2)
{
    TriangulateMapPoint(pMapPoint->position_, frame1->keypoints_[match.queryIdx], frame2->keypoints_[match.trainIdx],
            frame1->projectionMatrix_, frame2->projectionMatrix_);
}


int main(int argc, char** argv)
{
    if(argc < 2)
    {
        std::cout << "Usage: " << argv[0] << " config.yaml" << std::endl;
        return 1;
    }

    // ----------------------- 读取配置文件 ----------------------------
    cv::FileStorage fs;
    fs.open(string(argv[1]), cv::FileStorage::READ);
    if(!fs.isOpened())
    {
        cerr << "打不开配置文件: " << argv[1] << " !" << endl;
        return 1;
    }

    fs.release();

    // ------------------ 读取数据集 --------------------------------------
    ImageSourceManager imageSourceManager;
    imageSourceManager.LoadImageAndPose(argv[1]);

    std::string datasetPath = imageSourceManager.poseFilePath + "/";
    std::vector<std::pair<Eigen::Matrix3f,Eigen::Vector3f>> posesTwc = imageSourceManager.posesTwc;
    std::vector<std::string> rgbNames = imageSourceManager.rgbNames;
    std::vector<Intrinsic> intrinsics = imageSourceManager.intrinsics;

    FrameDataset frameDataset;
    Map currMap;

    // 特征点检测和描述子计算
//    cv::Ptr<cv::ORB> featureDetector = cv::ORB::create();
    cv::Ptr<cv::AKAZE> featureDetector = cv::AKAZE::create();
    for(size_t i=0, iEnd=rgbNames.size(); i<iEnd; ++i)
    {
        cv::Mat img = cv::imread(datasetPath+rgbNames[i], CV_LOAD_IMAGE_UNCHANGED);
        std::vector<cv::KeyPoint> keypoints;
        cv::Mat descriptors;
        featureDetector->detectAndCompute(img, cv::Mat(), keypoints, descriptors);

        cv::Mat imgWithFeature;
        cv::drawKeypoints(img, keypoints, imgWithFeature, cv::Scalar(0,0,255), cv::DrawMatchesFlags::DEFAULT);

        cv::imshow("Flann Matching Result", imgWithFeature);
        cv::waitKey(30);


        Eigen::Matrix4f poseTwc;
        std::pair<Eigen::Matrix3f, Eigen::Vector3f>& poseTwcOriginal = posesTwc[i];
        poseTwc.block(0, 0, 3, 3) = poseTwcOriginal.first;
        poseTwc.block(0, 3, 3, 1) = poseTwcOriginal.second;

        Intrinsic& intrinOriginal = intrinsics[i];
        Eigen::Vector4f intrin = Eigen::Vector4f(intrinOriginal.x, intrinOriginal.y, intrinOriginal.z, intrinOriginal.w);

        // 插入一帧
        Frame* pFrame(new Frame);
        pFrame->img_ = img;
        pFrame->keypoints_ = keypoints;
        pFrame->pMapPoints_.resize(keypoints.size(), NULL);
        pFrame->descriptors_ = descriptors.clone();
        pFrame->intrinsic_ = intrin;
        pFrame->poseTwc_ = poseTwc;
        frameDataset.AddFrame(pFrame);
    }

    //BruteForce match
//    cv::FlannBasedMatcher matcher;
    cv::BFMatcher matcher;
    std::vector<cv::DMatch> matches;
    std::vector<uchar> isMatchGood;
    for(size_t i=0, iEnd=frameDataset.size()-1; i<iEnd; ++i)
    {
        Frame* frame_i = frameDataset[i];
        for(size_t j=i+1, jEnd=frameDataset.size(); j<jEnd; ++j)
        {
            Frame* frame_j = frameDataset[j];
//            matcher.match(frame_i->descriptors_, frame_j->descriptors_, matches);
//            FilterOutlierByFMatrix(isMatchGood, matches, frame_i, frame_j);

            std::vector<std::vector<cv::DMatch>> knnMatches;
            matcher.knnMatch(frame_i->descriptors_, frame_j->descriptors_, knnMatches, 2);
            FindGoodMatches(isMatchGood, knnMatches, 0.7f);

//            cv::Mat matchImg;
//            cv::drawMatches(frame_i->img_, frame_i->keypoints_, frame_j->img_, frame_j->keypoints_,
//                            matches, matchImg);
//            cv::imshow("matches", matchImg);
//            cv::waitKey(0);

            std::vector<cv::DMatch> goodMatches;
//            for(size_t iii=0; iii<isMatchGood.size(); ++iii)
//            {
//                if(isMatchGood[iii] != 0)
//                    goodMatches.push_back(matches[iii]);
//            }

            for(size_t iii=0; iii<isMatchGood.size(); ++iii)
            {
                if(isMatchGood[iii] != 0)
                    goodMatches.push_back(knnMatches[iii][0]);
            }
            cv::Mat matchImg2;
            cv::drawMatches(frame_i->img_, frame_i->keypoints_, frame_j->img_, frame_j->keypoints_,
                            goodMatches, matchImg2);
            cv::imshow("matches2", matchImg2);
            cv::waitKey(0);

            for(int k=0, kEnd=goodMatches.size(); k<kEnd; ++k)
            {

                // 看 query 点是否已经是某个mappoint的观测了，如果不是就应该生成一个新的mappoint
                Frame* queryFrame = frame_i;
                Frame* trainFrame = frame_j;
                const cv::DMatch& match = goodMatches[k];
                size_t queryPointIdx = match.queryIdx;
                size_t trainPointIdx = match.trainIdx;
                if(queryFrame->pMapPoints_[queryPointIdx] != NULL)
                {
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
                    TriangulateMapPoint(pMapPoint, match, queryFrame, trainFrame);

                    // queryFrame 和 trainFrame 都增加一个地图点
                    queryFrame->pMapPoints_[queryPointIdx] = pMapPoint;
                    trainFrame->pMapPoints_[trainPointIdx] = pMapPoint;

                    // 该地图点增加两次观测
                    Observation* obs1(new Observation(trainFrame, trainPointIdx));
                    Observation* obs2(new Observation(queryFrame, queryPointIdx));
                    pMapPoint->AddObservation(obs1);
                    pMapPoint->AddObservation(obs2);

                    // 将地图点添加到地图中
                    currMap.AddMapPoint(pMapPoint);
                }
            }


        }
    }

    return 0;
}
