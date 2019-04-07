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

#include "reprojection_error_with_quaternion.h"

#include "SLAM/map.h"
#include "SLAM/map_point.h"
#include "SLAM/frame.h"
#include "SLAM/frame_database.h"
#include "SLAM/optimizer.h"
#include "SLAM/tracker.h"
#include "SLAM/slam_system.h"

using SLAM::Observation;
using SLAM::MapPoint;
using SLAM::Map;
using SLAM::Frame;
using SLAM::FrameDatabase;
using SLAM::Optimizer;
using SLAM::SLAM_System;


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

    FrameDatabase frameDatabase;
    Map currMap;
    currMap.pFrameDatabase_ = &frameDatabase;

    SLAM_System slamer;
    size_t iEnd = 100;
    iEnd = rgbNames.size() > iEnd ? iEnd : rgbNames.size();
    for(size_t i=0; i<iEnd; ++i)
    {
        cv::Mat img = cv::imread(datasetPath+rgbNames[i], CV_LOAD_IMAGE_UNCHANGED);
        std::pair<Eigen::Matrix3f, Eigen::Vector3f>& poseTwcOriginal = posesTwc[i];
        Intrinsic& intrinOriginal = intrinsics[i];
        Eigen::Vector4f intrin = Eigen::Vector4f(intrinOriginal.x, intrinOriginal.y, intrinOriginal.z, intrinOriginal.w);
        slamer.TrackMonocular(img, poseTwcOriginal.first, poseTwcOriginal.second, intrin);
    }

    slamer.CompleteTracking();
    slamer.SaveMapPoints("beforOptimize.ply");
    slamer.Optimize();
    slamer.SaveMapPoints("afterOptimize.ply");
    return 0;
}
