#ifndef IMAGE_SOURCE_MANAGER
#define IMAGE_SOURCE_MANAGER

#include <iostream>
#include <fstream>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "types_customized.h"

using namespace std;

class ImageSourceManager
{
public:
    string imgSource = "arkit"; // 图像来源，有 tum， arkit， arcore
    string poseFilePath;
    string poseFilename;
    int imgNumTotal;
    vector<pair<Eigen::Matrix3f,Eigen::Vector3f>> posesTwc;
    vector<string> rgbNames;
    vector<string> depthNames; // 深度图的名字，只有 tum 数据集有
    vector<Intrinsic> intrinsics;

    ImageSourceManager() {}

    int GetNextFrame(cv::Mat& img, ImagePoseStruct& currTwc, Intrinsic& intrin);

    void LoadImageAndPose(string configFilename);
    void LoadImageAndPose(string configFilename, string datasetPath);

    void LoadARkitImageAndPose(vector<pair<Eigen::Matrix3f,Eigen::Vector3f>>& posesFromFile, vector<Intrinsic>& intrinsics, vector<string>& rgbNames,
                          string poseFilePath, string poseFilename);

    void LoadARcoreImageAndPose(vector<pair<Eigen::Matrix3f,Eigen::Vector3f>>& posesFromFile, vector<Intrinsic>& intrinsics, vector<string>& rgbNames,
                          string poseFilePath, string poseFilename);

    void LoadTumImageAndPose(vector<pair<Eigen::Matrix3f,Eigen::Vector3f>>& posesFromFile, vector<string>& depthNames, vector<string>& rgbNames,
                          string poseFilePath, string poseFilename);

    template<typename T>
    void ReadParamCVFs(T& param, cv::FileStorage fs, string paramname)
    {
        cv::FileNode node = fs[paramname];
        if(!node.empty())
            fs[paramname] >> param;
    }

};


int ImageSourceManager::GetNextFrame(cv::Mat &img, ImagePoseStruct &currTwc, Intrinsic &intrin)
{
    static int currIndex = 0;
    if(currIndex < imgNumTotal)
    {
        string rgbName = rgbNames[currIndex];
        img = cv::imread(poseFilePath+"/"+rgbName, CV_LOAD_IMAGE_UNCHANGED);
        currTwc = ImagePoseStruct(currIndex, posesTwc[currIndex]);
        intrin = intrinsics[currIndex];

        currIndex++;
    }
    else
        return -1;

    if(img.empty())
        return -1;
    else
        return 0;
}


void ImageSourceManager::LoadImageAndPose(string configFilename)
{
    // ----------------------- 读取配置文件 ----------------------------
    cv::FileStorage fs;
    fs.open(string(configFilename), cv::FileStorage::READ);
    if(!fs.isOpened())
    {
        cerr << "打不开配置文件: " << configFilename << " !" << endl;
        return;
    }

    float fx = 525.0f;
    float fy = 525.0f;
    float cx = 319.5f;
    float cy = 239.5f;

    string poseFilePath_ = "/media/zcc/DVolume/zcc_work/dataset/TUM/RGBD/rgbd_dataset_freiburg2_desk";
    string poseFilename_ = "associations_orbpose_depth_rgb.txt";

    ReadParamCVFs(poseFilePath_, fs, "poseFilePath");
    ReadParamCVFs(poseFilename_, fs, "poseFilename");
    ReadParamCVFs(imgSource, fs, "imgSource");

    ReadParamCVFs(fx, fs, "fx");
    ReadParamCVFs(fy, fs, "fy");
    ReadParamCVFs(cx, fs, "cx");
    ReadParamCVFs(cy, fs, "cy");

    fs.release();

    poseFilename = poseFilename_;
    poseFilePath = poseFilePath_;


    if(!imgSource.compare("arkit"))
    {
        LoadARkitImageAndPose(posesTwc, intrinsics, rgbNames, poseFilePath, poseFilename);
    }
    else if(!imgSource.compare("arcore"))
    {
        LoadARcoreImageAndPose(posesTwc, intrinsics, rgbNames, poseFilePath, poseFilename);
    }
    else if(!imgSource.compare("tum"))
    {
        LoadTumImageAndPose(posesTwc, depthNames, rgbNames, poseFilePath, poseFilename);
        for(int i=0, i_end=posesTwc.size(); i<i_end; i++)
        {
            intrinsics.push_back(Intrinsic(fx, fy, cx, cy));
        }
    }

    imgNumTotal = posesTwc.size();
}

void ImageSourceManager::LoadImageAndPose(string configFilename, string datasetPath)
{
    // ----------------------- 读取配置文件 ----------------------------
    cv::FileStorage fs;
    fs.open(string(configFilename), cv::FileStorage::READ);
    if(!fs.isOpened())
    {
        cerr << "打不开配置文件: " << configFilename << " !" << endl;
        return;
    }
    
    float fx = 525.0f;
    float fy = 525.0f;
    float cx = 319.5f;
    float cy = 239.5f;
    
//    string poseFilePath_ = "/media/zcc/DVolume/zcc_work/dataset/TUM/RGBD/rgbd_dataset_freiburg2_desk";
    string poseFilePath_ = datasetPath;
    string poseFilename_ = "arkit.csv";
    
//    ReadParamCVFs(poseFilePath_, fs, "poseFilePath");
    ReadParamCVFs(poseFilename_, fs, "poseFilename");
    
    ReadParamCVFs(fx, fs, "fx");
    ReadParamCVFs(fy, fs, "fy");
    ReadParamCVFs(cx, fs, "cx");
    ReadParamCVFs(cy, fs, "cy");
    
    fs.release();
    
    poseFilename = poseFilename_;
    poseFilePath = poseFilePath_;
    
    
    if(!imgSource.compare("arkit"))
    {
        LoadARkitImageAndPose(posesTwc, intrinsics, rgbNames, poseFilePath, poseFilename);
    }
    else if(!imgSource.compare("arcore"))
    {
        LoadARcoreImageAndPose(posesTwc, intrinsics, rgbNames, poseFilePath, poseFilename);
    }
    else if(!imgSource.compare("tum"))
    {
        LoadTumImageAndPose(posesTwc, depthNames, rgbNames, poseFilePath, poseFilename);
        for(int i=0, i_end=posesTwc.size(); i<i_end; i++)
        {
            intrinsics.push_back(Intrinsic(fx, fy, cx, cy));
        }
    }
    
    imgNumTotal = posesTwc.size();
}

void ImageSourceManager::LoadARkitImageAndPose(vector<pair<Eigen::Matrix3f, Eigen::Vector3f> > &posesFromFile, vector<Intrinsic> &intrinsics, vector<string> &rgbNames, string poseFilePath, string poseFilename)
{
    ifstream fs;
    fs.open(poseFilePath+"/"+poseFilename);
    if(!fs.is_open())
    {
        cerr << "Error! Cannot open file: " << poseFilePath << "/" << poseFilename << endl;
        return;
    }
    string lineData;
    string imageName;
    int trackState;
    float fx, fy, cx, cy;
    float tx, ty, tz;
    float rx, ry, rz, rw;
    while(!fs.eof())
    {
        getline(fs, lineData);
        istringstream sin(lineData);
        sin >> imageName >> trackState >> fx >> fy >> cx >> cy >> tx >> ty >> tz >> rw >> rx >> ry >> rz;
        if(trackState != 2)
            continue;

        rgbNames.push_back(imageName);
        Intrinsic intri(fx, fy, cx, cy);
        intrinsics.push_back(intri);

        Eigen::Matrix3f R = Eigen::Quaternionf(rw, rx, ry, rz).toRotationMatrix();
        Eigen::Matrix3f temp;
        temp << 1, 0, 0, 0, -1, 0, 0, 0, -1;
        R = R * temp;
        Eigen::Vector3f t = Eigen::Vector3f(tx,ty,tz);
        posesFromFile.push_back(std::pair<Eigen::Matrix3f,Eigen::Vector3f>(R,t));
    }

    fs.close();
}

void ImageSourceManager::LoadARcoreImageAndPose(vector<pair<Eigen::Matrix3f, Eigen::Vector3f> > &posesFromFile, vector<Intrinsic> &intrinsics, vector<string> &rgbNames, string poseFilePath, string poseFilename)
{
    fstream fs;

    fs.open(poseFilePath+"/"+poseFilename);
    if(!fs.is_open())
    {
        cerr << "Error! Cannot open file: " << poseFilePath << "/" << poseFilename << endl;
        return;
    }

    string lineData;        // 每一行内容的缓存
    stringstream ss;            // 转换器

    int numOfFrame;  // 深度图数量
    string rgbName;

    getline(fs, lineData); // 有两行没有用的
    getline(fs, lineData);

    getline(fs, lineData);
    ss << lineData;
    ss >> numOfFrame;
    ss.clear(); ss.str("");

    posesFromFile.resize(numOfFrame);
    rgbNames.resize(numOfFrame);
    intrinsics.resize(numOfFrame);
    Eigen::Quaternionf r_cw;
    Eigen::Matrix3f R_cw;
    Eigen::Vector3f center; // t_cw
    Eigen::Vector3f t_wc;
    Eigen::Matrix3f R_wc;
    Intrinsic intrin;
    float temp;

    float focal;
    for(int i=0; i<numOfFrame; i++)
    {
        getline(fs, lineData);
        std::stringstream ss(lineData);
        ss << lineData;
        ss >> rgbName;
        ss >> focal;

        ss >> r_cw.w();
        ss >> r_cw.x();
        ss >> r_cw.y();
        ss >> r_cw.z();

        ss >> center[0];
        ss >> center[1];
        ss >> center[2];

        ss >> temp;
        ss >> temp;
        if(ss >> temp)   // 新的 nvm 文件
        {
            intrin.x = temp;
            ss >> intrin.y;
            ss >> intrin.z;
            ss >> intrin.w;

            ss >> temp;
            ss >> temp;
        }
        else                // 旧的 nvm 文件，没有记录内参，所以直接用图像的一半代替cx， cy
        {
            static int flag = 0;
            static float cx = 320;
            static float cy = 240;
            if(flag == 0)
            {
                cv::Mat img = cv::imread(poseFilePath+"/"+rgbName,0);
                cx = img.cols / 2;
                cy = img.rows / 2;
                flag = 1;
            }
            intrin.x = focal;
            intrin.y = focal;
            intrin.z = cx;
            intrin.w = cy;
        }

        R_cw = r_cw.matrix();
        R_wc = R_cw.transpose();
//        R_wc = R_cw;

//        t_wc = -R_cw.transpose() * center;
        t_wc = center;
        pair<Eigen::Matrix3f,Eigen::Vector3f> pose(R_wc, t_wc);

        rgbNames[i] = rgbName;
        posesFromFile[i] = pose;
        intrinsics[i] = intrin;
    }
    fs.close();
}


void ImageSourceManager::LoadTumImageAndPose(vector<pair<Eigen::Matrix3f, Eigen::Vector3f> > &posesFromFile, vector<string> &depthNames, vector<string> &rgbNames, string poseFilePath, string poseFilename)
{
    fstream fs;
    fs.open(poseFilePath+"/"+poseFilename);
    if(!fs.is_open())
    {
        cerr << "没能打开 association 文件: " << poseFilePath << poseFilename << " !" << endl;
        return;
    }

    double junkstamp;
    double tx, ty, tz, rw, rx, ry, rz;

    string depthName, rgbName;

    while(!fs.eof())
    {
        std::string temp("");
        getline(fs,temp);
        std::stringstream stream(temp);
        stream >> junkstamp;
        stream >> tx; stream >> ty; stream >> tz;
        stream >> rx; stream >> ry; stream >> rz; stream >> rw;
        stream >> junkstamp;
        stream >> depthName;
        if(stream >> junkstamp)
        {
            Eigen::Matrix3f R = Eigen::Quaternionf(rw, rx, ry, rz).toRotationMatrix();
            Eigen::Vector3f t = Eigen::Vector3f(tx,ty,tz);
            posesFromFile.push_back(std::pair<Eigen::Matrix3f,Eigen::Vector3f>(R,t));
            depthNames.push_back(depthName);

            stream >> rgbName;
            rgbNames.push_back(rgbName);
        }
    }

    fs.close();
}


#endif // IMAGE_SOURCE_MANAGER
