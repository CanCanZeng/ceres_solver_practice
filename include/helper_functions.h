#pragma once
#ifndef HELPER_FUNCTIONS_H
#define HELPER_FUNCTIONS_H

#include <iostream>
#include <fstream>
#include <list>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
//#include <opencv/cxeigen.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "types_customized.h"

using namespace std;

template<typename T>
__inline
void ReadParamCVFs(T& param, cv::FileStorage fs, string paramname)
{
    cv::FileNode node = fs[paramname];
    if(!node.empty())
        fs[paramname] >> param;
}

inline
void LoadImageAndPose(vector<pair<Eigen::Matrix3f,Eigen::Vector3f>>& poses_from_file, vector<string>& depth_names, vector<string>& rgb_names,
                      string association_file_path, string association_file_name)
{
    fstream fs;
    fs.open(association_file_path+"/"+association_file_name);
    if(!fs.is_open())
    {
        cerr << "没能打开 association 文件: " << association_file_path << association_file_name << " !" << endl;
        return;
    }

    double junkstamp;
    double tx, ty, tz, rw, rx, ry, rz;

    string depth_name, rgb_name;

    while(!fs.eof())
    {
        std::string temp("");
        getline(fs,temp);
        std::stringstream stream(temp);
        stream >> junkstamp;
        stream >> tx; stream >> ty; stream >> tz;
        stream >> rx; stream >> ry; stream >> rz; stream >> rw;
        stream >> junkstamp;
        stream >> depth_name;
        if(temp!="")
        {
            Eigen::Matrix3f R = Eigen::Quaternionf(rw, rx, ry, rz).toRotationMatrix();
            Eigen::Vector3f t = Eigen::Vector3f(tx,ty,tz);
            poses_from_file.push_back(std::pair<Eigen::Matrix3f,Eigen::Vector3f>(R,t));
            depth_names.push_back(depth_name);

            stream >> junkstamp; stream >> rgb_name;
            rgb_names.push_back(rgb_name);
        }
    }
}



// 从一个三维数组保存点云
inline
bool SavePlyFile(string filename, Float3* point_cloud_data, int point_num)
{
    FILE *fp = NULL;
    fp = fopen(filename.c_str(), "w");

    if(NULL == fp) // 判断文件打开是否成功
        return false;
    fprintf(fp, "ply\n");
    fprintf(fp, "format binary_little_endian 1.0\n");
    fprintf(fp, "element vertex %d\n", point_num);
    fprintf(fp, "property float x\n");
    fprintf(fp, "property float y\n");
    fprintf(fp, "property float z\n");
    fprintf(fp, "end_header\n");

    // Create point cloud content for ply file
    for (int i = 0; i < point_num; i++) {
        // Convert voxel indices to float, and save coordinates to ply file
        Float3 pointXYZ = point_cloud_data[i];
        float pt_base_x = pointXYZ.x;
        float pt_base_y = pointXYZ.y;
        float pt_base_z = pointXYZ.z;

        if(!(pt_base_x < 10 && pt_base_x > -10 &&
                pt_base_y < 10 && pt_base_y > -10 &&
                pt_base_z < 10 && pt_base_z > -10))
        {
            pt_base_x = pt_base_y = pt_base_z = 0.0f;
        }

        fwrite(&pt_base_x, sizeof(float), 1, fp);
        fwrite(&pt_base_y, sizeof(float), 1, fp);
        fwrite(&pt_base_z, sizeof(float), 1, fp);
        }
    fclose(fp);

    return true;
}


inline
bool SavePlyFile(string filename, Float3* point_cloud_data, Uchar3* color_data, int point_num)
{
    FILE *fp = NULL;
    fp = fopen(filename.c_str(), "w");

    if(NULL == fp) // 判断文件打开是否成功
        return false;
    fprintf(fp, "ply\n");
    fprintf(fp, "format binary_little_endian 1.0\n");
    fprintf(fp, "element vertex %d\n", point_num);
    fprintf(fp, "property float x\n");
    fprintf(fp, "property float y\n");
    fprintf(fp, "property float z\n");
    fprintf(fp, "property uchar red\n");
    fprintf(fp, "property uchar green\n");
    fprintf(fp, "property uchar blue\n");
    fprintf(fp, "property uchar alpha\n");
    fprintf(fp, "end_header\n");

    // Create point cloud content for ply file
    for (int i = 0; i < point_num; i++) {
        // Convert voxel indices to float, and save coordinates to ply file
        Float3 pointXYZ = point_cloud_data[i];
        float pt_base_x = pointXYZ.x;
        float pt_base_y = pointXYZ.y;
        float pt_base_z = pointXYZ.z;

        if(!(pt_base_x < 10 && pt_base_x > -10 &&
                pt_base_y < 10 && pt_base_y > -10 &&
                pt_base_z < 10 && pt_base_z > -10))
        {
            pt_base_x = pt_base_y = pt_base_z = 0.0f;
        }

        Uchar3 color = color_data[i];
        uchar color_r = color.x;
        uchar color_g = color.y;
        uchar color_b = color.z;
        uchar color_a = 255;

        fwrite(&pt_base_x, sizeof(float), 1, fp);
        fwrite(&pt_base_y, sizeof(float), 1, fp);
        fwrite(&pt_base_z, sizeof(float), 1, fp);
        fwrite(&color_r, sizeof(uchar), 1, fp);
        fwrite(&color_g, sizeof(uchar), 1, fp);
        fwrite(&color_b, sizeof(uchar), 1, fp);
        fwrite(&color_a, sizeof(uchar), 1, fp);
        }
    fclose(fp);

    return true;
}


inline
bool SavePlyFile(string filename, PointXYZRGB* point_cloud_data, int point_num)
{
    FILE *fp = NULL;
    fp = fopen(filename.c_str(), "w");

    if(NULL == fp) // 判断文件打开是否成功
        return false;
    fprintf(fp, "ply\n");
    fprintf(fp, "format binary_little_endian 1.0\n");
    fprintf(fp, "element vertex %d\n", point_num);
    fprintf(fp, "property float x\n");
    fprintf(fp, "property float y\n");
    fprintf(fp, "property float z\n");
    fprintf(fp, "property uchar red\n");
    fprintf(fp, "property uchar green\n");
    fprintf(fp, "property uchar blue\n");
    fprintf(fp, "property uchar alpha\n");
    fprintf(fp, "end_header\n");

    // Create point cloud content for ply file
    for (int i = 0; i < point_num; i++) {
        // Convert voxel indices to float, and save coordinates to ply file
        PointXYZRGB pointXYZRGB = point_cloud_data[i];
        float pt_base_x = pointXYZRGB.pt_.x;
        float pt_base_y = pointXYZRGB.pt_.y;
        float pt_base_z = pointXYZRGB.pt_.z;

        if(!(pt_base_x < 10 && pt_base_x > -10 &&
                pt_base_y < 10 && pt_base_y > -10 &&
                pt_base_z < 10 && pt_base_z > -10))
        {
            pt_base_x = pt_base_y = pt_base_z = 0.0f;
        }

        uchar color_r = pointXYZRGB.color_.x;
        uchar color_g = pointXYZRGB.color_.y;
        uchar color_b = pointXYZRGB.color_.z;
        uchar color_a = 255;

        fwrite(&pt_base_x, sizeof(float), 1, fp);
        fwrite(&pt_base_y, sizeof(float), 1, fp);
        fwrite(&pt_base_z, sizeof(float), 1, fp);
        fwrite(&color_r, sizeof(uchar), 1, fp);
        fwrite(&color_g, sizeof(uchar), 1, fp);
        fwrite(&color_b, sizeof(uchar), 1, fp);
        fwrite(&color_a, sizeof(uchar), 1, fp);
        }
    fclose(fp);

    return true;
}


// 从一个向量保存点云
inline
bool SavePlyFile(string filename, vector<cv::Point3f> point_cloud_data)
{
    FILE *fp = NULL;
    fp = fopen(filename.c_str(), "w");

    int point_num = point_cloud_data.size();

    if(NULL == fp) // 判断文件打开是否成功
        return false;
    fprintf(fp, "ply\n");
    fprintf(fp, "format binary_little_endian 1.0\n");
    fprintf(fp, "element vertex %d\n", point_num);
    fprintf(fp, "property float x\n");
    fprintf(fp, "property float y\n");
    fprintf(fp, "property float z\n");
    fprintf(fp, "end_header\n");

    // Create point cloud content for ply file
    for (int i = 0; i < point_num; i++) {
        // Convert voxel indices to float, and save coordinates to ply file
        cv::Point3f pointXYZ = point_cloud_data[i];
        float pt_base_x = pointXYZ.x;
        float pt_base_y = pointXYZ.y;
        float pt_base_z = pointXYZ.z;

        if(!(pt_base_x < 10 && pt_base_x > -10 &&
                pt_base_y < 10 && pt_base_y > -10 &&
                pt_base_z < 10 && pt_base_z > -10))
        {
            pt_base_x = pt_base_y = pt_base_z = 0.0f;
        }

        fwrite(&pt_base_x, sizeof(float), 1, fp);
        fwrite(&pt_base_y, sizeof(float), 1, fp);
        fwrite(&pt_base_z, sizeof(float), 1, fp);
        }
    fclose(fp);

    return true;
}


inline
bool SavePlyFile(string filename, vector<Eigen::Vector3f> point_cloud_data)
{
    std::ofstream stream(filename.c_str());
    if (!stream)
    {
        return false;
    }
    
    size_t numPoints = point_cloud_data.size();
    stream << "ply" << std::endl;
    stream << "format ascii 1.0" << std::endl;
    stream << "element vertex " << numPoints << std::endl;
    stream << "property float x" << std::endl;
    stream << "property float y" << std::endl;
    stream << "property float z" << std::endl;
    stream << "element face " << numPoints / 3 <<  std::endl;
    stream << "property list uchar int vertex_index" << std::endl;
    stream << "end_header" << std::endl;
    
    for (const Eigen::Vector3f& vert : point_cloud_data)
    {
        stream << vert[0] << " " << vert[1] << " " << vert[2];
        
        stream << std::endl;
    }
    
    for (size_t i = 0; i < numPoints; i++)
    {
        stream << "3 ";
        stream << i*3 << " " << i*3+1 << " " << i*3+2;
        
        stream << std::endl;
    }
    std::cout << "saved local mesh!" << std::endl;
    
    return true;
}


// 将光流转换成深度图
// flow 是 CV_32FC2的， depth是 CV_32FC1的， R21和t21 是第1帧到第2帧的变换
// threshold是对结果进行检查时，将1的深度图转换成点云投到2中，投影点的位置和光流得到的对应点的位置之间允许的最大像素偏差
// K是内参
// 根据计算公式应该有 s2 * x2 = s1 * R21 * x1 + t21
inline
void Flow2Depth(cv::Mat& depth, cv::Mat& flow, cv::Mat R21, cv::Mat t21, float fx,
                float fy, float cx, float cy, float threshold)
{
    float f = (fx + fy) / 2.0f;
    R21.convertTo(R21, CV_32FC1);
    t21.convertTo(t21, CV_32FC1);
    for(int i=0; i<flow.rows; i++)
    {
        for(int j=0; j<flow.cols; j++)
        {
            float p2_x = j + flow.at<cv::Vec2f>(i,j)[0];
            float p2_y = i + flow.at<cv::Vec2f>(i,j)[1];
            if(p2_x<0 || p2_x>flow.cols || p2_y<0 || p2_y>flow.rows)
            {
                depth.at<float>(i,j) = 0;
                continue;
            }
            else
            {
                cv::Mat x1 = (cv::Mat_<float>(3,1) << (j-cx)/f, (i-cy)/f, 1);
                cv::Mat x2 = (cv::Mat_<float>(3,1) << (p2_x-cx)/f, (p2_y-cy)/f, 1);
                cv::Mat x2_skew = (cv::Mat_<float>(3,3) << 0, -x2.at<float>(2,0), x2.at<float>(1,0),
                                                            x2.at<float>(2,0), 0, -x2.at<float>(0,0),
                                                            -x2.at<float>(1,0), x2.at<float>(0,0), 0);
                cv::Mat a = x2_skew * R21 * x1;
                cv::Mat b = t21;

                cv::Mat s = - a.t() * b / (a.t()*a);
                float d = s.at<float>(0,0);

                cv::Mat P2 = d * R21 * x1 + t21;
                float x = P2.at<float>(0,0), y = P2.at<float>(1,0), z = P2.at<float>(2,0);
                float p2_x_proj = x/z*f+cx;
                float p2_y_proj = y/z*f+cy;
                if(sqrt((p2_x_proj-p2_x)*(p2_x_proj-p2_x)+(p2_y_proj-p2_y)*(p2_y_proj-p2_y)) > threshold)
                    d = 0;
                depth.at<float>(i,j) = d;
            }
        }
    }
}


inline
void Flow2Depth(cv::Mat& depth, cv::Mat& flow, Eigen::Matrix3f R21, Eigen::Vector3f t21, float fx,
                float fy, float cx, float cy, float threshold, int border_width=10, int sample_rate=5)
{
    int border_left = border_width;
    int border_right = flow.rows - border_width;
    int border_up = border_width;
    int border_down = flow.rows - border_width;
    float f = (fx + fy) / 2.0f;
    Eigen::Matrix3f K;
    K << fx, 0, cx, 0, fy, cy, 0, 0, 1;
    Eigen::Matrix3f KR21K_ = K * R21 * K.inverse();
    Eigen::Vector3f Kt21 = K * t21;

    for(int i=border_up; i<border_down; i+=sample_rate)
    {
        for(int j=border_left; j<border_right; j+=sample_rate)
        {
            cv::Vec2f temp = flow.at<cv::Vec2f>(i, j);
            cv::Vec2f pt2(temp[0]+j, temp[1]+i);
            if(pt2[0] < border_left || pt2[0] > border_right || pt2[1] < border_up || pt2[1] > border_down
                   )
//                 || (fabs(temp[0]) < 1 && fabs(temp[1]) < 1)
            {
//                depth.at<float>(i,j) = 0;
                continue;
            }
            else
            {
                Eigen::Vector3f x1(j, i, 1);
                Eigen::Vector3f x2(pt2[0], pt2[1], 1);
                Eigen::Matrix3f x2_skew;
                x2_skew << 0, -x2[2], x2[1],
                        x2[2], 0, -x2[0],
                        -x2[1], x2[0], 0;

                Eigen::Vector3f a = x2_skew * KR21K_ * x1;
                Eigen::Vector3f b = x2_skew * Kt21;

                auto s = - a.transpose() * b / (a.transpose()*a);
                float d = s[0];

//                Eigen::Vector3f P2 = d * R21 * x1 + t21;
//                float x = P2[0], y = P2[1], z = P2[2];
//                float p2_x_proj = x/z*f+cx;
//                float p2_y_proj = y/z*f+cy;
                Eigen::Vector3f p2 = d * KR21K_ * x1 + Kt21;
                float p2_x_proj = p2[0]/p2[2];
                float p2_y_proj = p2[1]/p2[2];
                if(sqrt((p2_x_proj-pt2[0])*(p2_x_proj-pt2[0])+(p2_y_proj-pt2[1])*(p2_y_proj-pt2[1])) > threshold
                        || d < 0.2 || d > 3.0)
                    d = 0;
                depth.at<float>(i,j) = d;
            }
        }
    }
}


inline
void Depthmap2PC(Float3* pc, cv::Mat depthmap, float fx, float fy, float cx, float cy, float scale)
{
    for(int i=0; i < depthmap.rows; i++)
        for(int j=0; j<depthmap.cols; j++)
        {
            float z = depthmap.at<float>(i, j);
            z /= scale;
            float x = z / fx * (j - cx);
            float y = z / fy * (i - cy);
            pc[i*depthmap.cols+j].x = x;
            pc[i*depthmap.cols+j].y = y;
            pc[i*depthmap.cols+j].z = z;
        }
}


inline
void Depthmap2PC(PointXYZRGB* pc, cv::Mat depthmap, cv::Mat colormap, float fx, float fy,
                 float cx, float cy, float scale)
{
    for(int i=0; i < depthmap.rows; i++)
        for(int j=0; j<depthmap.cols; j++)
        {
            float z = depthmap.at<float>(i, j);
            z /= scale;
            float x = z / fx * (j - cx);
            float y = z / fy * (i - cy);
            Float3 pt(x, y, z);
            cv::Vec3b color = colormap.at<cv::Vec3b>(i,j);
            pc[i*depthmap.cols+j].pt_ = pt;
            pc[i*depthmap.cols+j].color_ = Uchar3(color[2], color[1], color[0]);
        }
}

//
//inline
//void Flow2CVPointcloud(cv::Mat& cloud_xyz, cv::Mat& cloud_rgb, cv::Mat& flow, cv::Mat ref_image_color, Eigen::Matrix3f R21, Eigen::Vector3f t21, float fx,
//                float fy, float cx, float cy, int border_width=5, int sample_rate=5)
//{
//    int w = flow.cols / sample_rate;
//    int h = flow.rows / sample_rate;
//    int total = w * h;
//    cv::Mat pts3D(4, total, CV_64FC1);  // 3D 点
//    cv::Mat cam1pts(2, total, CV_64FC1);   // 图像0 中的 2D 点
//    cv::Mat cam2pts(2, total, CV_64FC1);   // 图像1 中的 2D 点
//    cv::Mat P1(3, 4, CV_64FC1), P2(3, 4, CV_64FC1);   // 相机投影矩阵，包含了内参和外参
//
//    cv::Mat cam1, cam2, distort1, distort2;
//    cam1 = (cv::Mat_<double>(3,3) << fx, 0 , cx, 0, fy, cy, 0, 0, 1);
//    cam2 = cam1.clone();
//
//    cv::Mat R;  // 两个相机的相对旋转
//    cv::Mat T;  // 两个相机的相对平移
//    cv::eigen2cv(R21, R);
//    cv::eigen2cv(t21, T);
//
//    cv::Mat R_T_1 = (cv::Mat_<double>(3, 4) << 1, 0, 0, 0,
//                                               0, 1, 0, 0,
//                                               0, 0, 1, 0);
//    cv::Mat R_T_2(3, 4, CV_64FC1);
//    R.copyTo(R_T_2(cv::Range(0, 3), cv::Range(0,3)));
//    T.copyTo(R_T_2(cv::Range(0,3), cv::Range(3,4)));
//
//    P1 = cam1 * R_T_1;
//    P2 = cam2 * R_T_2;
//
//    // ------------ 计算2D特征点位置 ---------------------
//    int  ind = 0;
//    int border_left = border_width;
//    int border_right = flow.rows - border_width;
//    int border_up = border_width;
//    int border_down = flow.rows - border_width;
//    for(int i=border_up; i<border_down; i+=sample_rate)
//    {
//        for(int j=border_left; j<border_right; j+=sample_rate)
//        {
//            cv::Vec2f temp = flow.at<cv::Vec2f>(i, j);
//            cv::Vec2f pt2(temp[0]+j, temp[1]+i);
//
//            if(pt2[0] > border_left && pt2[0] < border_right && pt2[1] > border_up && pt2[1] < border_down
//                    && (fabs(temp[0]) > 1 || fabs(temp[1]) > 1))
//            {
//                cam1pts.at<double>(0,ind) = j;
//                cam1pts.at<double>(1,ind) = i;
//                cam2pts.at<double>(0,ind) = j+temp[0];
//                cam2pts.at<double>(1,ind) = i+temp[1];
//                ind++;
//            }
//        }
//    }
//
//    ind -= 1; // 最后 ind++ 了一下，所以应该减 1
//
//    cv::triangulatePoints(P1, P2, cam1pts, cam2pts, pts3D);
//
//    // 复制数据
//    //    cloud_xyz.resize(cv::Size(3, total));
//    //    cloud_bgr.resize(cv::Size(3, total));
//    cloud_xyz = cv::Mat(3, total, CV_32FC1, cv::Scalar(0));
//    cloud_rgb = cv::Mat(3, total, CV_8UC1, cv::Scalar(0));
//
//    for(int i=0; i<ind; i++)
//    {
//        int x = cam1pts.at<double>(0,i);
//        int y = cam1pts.at<double>(1,i);
//        cv::Vec3b color = ref_image_color.at<cv::Vec3b>(y, x);
//        cloud_rgb.at<uchar>(0, i) = color[2];
//        cloud_rgb.at<uchar>(1, i) = color[1];
//        cloud_rgb.at<uchar>(2, i) = color[0];
//    }
//
//    for(int i=0; i<total; i++)
//    {
//        double x = pts3D.at<double>(0,i);
//        double y = pts3D.at<double>(1,i);
//        double z = pts3D.at<double>(2,i);
//        double w = pts3D.at<double>(3,i);
//        if(z/w < 0.3f || z/w > 3.0f)
//            continue;
//        cloud_xyz.at<float>(0, i) = x/w;
//        cloud_xyz.at<float>(1, i) = y/w;
//        cloud_xyz.at<float>(2, i) = z/w;
//    }
//}


inline
void ComputeMaskOfLowTextureRegion(cv::Mat& mask, cv::Mat image_gray)
{
    image_gray.convertTo(image_gray, CV_32FC1);
    cv::Mat edge_h, edge_v;
    cv::Mat kernel = (cv::Mat_<float>(3,3) << 1, 0 , -1, 2, 0, -2, 1, 0, -1)/8.0;
    cv::filter2D(image_gray, edge_h, image_gray.depth(), kernel);
    cv::filter2D(image_gray, edge_v, image_gray.depth(), kernel.t());

    cv::Mat edge = cv::abs(edge_h) + cv::abs(edge_v);
    cv::boxFilter(edge, edge, edge.depth(), cv::Size(10, 10));
    cv::Scalar sTemp = cv::sum(edge);
    float textureness = sTemp.val[0]/image_gray.cols/image_gray.rows;
    mask = edge < (textureness * 0.1f);
}

//
//inline
//void SparseFlow2CVPointcloud(cv::Mat& cloud_xyz, cv::Mat& cloud_rgb, vector<cv::Point2f>& pts1, vector<cv::Point2f>& pts2, vector<uchar>& status,
//                             cv::Mat& ref_image_color, Eigen::Matrix3f& R21, Eigen::Vector3f& t21,
//                             float fx, float fy, float cx, float cy, int border_width=5)
//{
//    int total = pts1.size();
//    cv::Mat pts3D(4, total, CV_64FC1);  // 3D 点
//    cv::Mat cam1pts(2, total, CV_64FC1);   // 图像0 中的 2D 点
//    cv::Mat cam2pts(2, total, CV_64FC1);   // 图像1 中的 2D 点
//    cv::Mat P1(3, 4, CV_64FC1), P2(3, 4, CV_64FC1);   // 相机投影矩阵，包含了内参和外参
//
//    cv::Mat cam1, cam2, distort1, distort2;
//    cam1 = (cv::Mat_<double>(3,3) << fx, 0 , cx, 0, fy, cy, 0, 0, 1);
//    cam2 = cam1.clone();
//
//    cv::Mat R;  // 两个相机的相对旋转
//    cv::Mat T;  // 两个相机的相对平移
//    cv::eigen2cv(R21, R);
//    cv::eigen2cv(t21, T);
//
//    cv::Mat R_T_1 = (cv::Mat_<double>(3, 4) << 1, 0, 0, 0,
//                                               0, 1, 0, 0,
//                                               0, 0, 1, 0);
//    cv::Mat R_T_2(3, 4, CV_64FC1);
//    R.copyTo(R_T_2(cv::Range(0, 3), cv::Range(0,3)));
//    T.copyTo(R_T_2(cv::Range(0,3), cv::Range(3,4)));
//
//    P1 = cam1 * R_T_1;
//    P2 = cam2 * R_T_2;
//
//    double *data1x, *data1y, *data2x, *data2y;
//    data1x = cam1pts.ptr<double>(0);
//    data1y = cam1pts.ptr<double>(1);
//    data2x = cam2pts.ptr<double>(0);
//    data2y = cam2pts.ptr<double>(1);
//    for(int i=0; i<total; i++)
//    {
//
//        data1x[i] = pts1[i].x;
//        data1y[i] = pts1[i].y;
//        data2x[i] = pts2[i].x;
//        data2y[i] = pts2[i].y;
//    }
//
//    cv::triangulatePoints(P1, P2, cam1pts, cam2pts, pts3D);
//
//    // 复制数据
//    cloud_xyz = cv::Mat(total, 3, CV_32FC1, cv::Scalar(0));
//    cloud_rgb = cv::Mat(total, 3, CV_8UC1, cv::Scalar(0));
//
//    data1x = cam1pts.ptr<double>(0);
//    data1y = cam1pts.ptr<double>(1);
//    uchar *data_rgb;
//    for(int i=0; i<total; i++)
//    {
//        if(status[i])
//        {
//            data_rgb = cloud_rgb.ptr<uchar>(i);
//            int x = data1x[i];
//            int y = data1y[i];
//            cv::Vec3b color = ref_image_color.at<cv::Vec3b>(y, x);
//            data_rgb[0] = color[2];
//            data_rgb[1] = color[1];
//            data_rgb[2] = color[0];
//        }
//    }
//
//    double *data_x, *data_y, *data_z, *data_w;
//    float *data_cloud_xyz;
//    data_x = pts3D.ptr<double>(0);
//    data_y = pts3D.ptr<double>(1);
//    data_z = pts3D.ptr<double>(2);
//    data_w = pts3D.ptr<double>(3);
//    for(int i=0; i<total; i++)
//    {
//        if(status[i])
//        {
//            data_cloud_xyz = cloud_xyz.ptr<float>(i);
//            double x = data_x[i];
//            double y = data_y[i];
//            double z = data_z[i];
//            double w = data_w[i];
//            if(z/w < 0.2 || z/w > 4.0)
//                continue;
//            data_cloud_xyz[0] = x/w;
//            data_cloud_xyz[1] = y/w;
//            data_cloud_xyz[2] = z/w;
//        }
//    }
//}


inline
void SparseFlow2CVPointcloud(cv::Mat& cloud_xyz, cv::Mat& cloud_rgb, vector<cv::Point2f>& pts1, vector<cv::Point2f>& pts2, vector<uchar>& status,
                             cv::Mat& ref_image_color, Eigen::Matrix3f& R_1w, Eigen::Vector3f& t_1w, Eigen::Matrix3f& R_2w, Eigen::Vector3f t_2w,
                             float fx, float fy, float cx, float cy, int border_width=5)
{
    int total = pts1.size();
    cv::Mat pts3D(4, total, CV_64FC1);  // 3D 点
    cv::Mat cam1pts(2, total, CV_64FC1);   // 图像0 中的 2D 点
    cv::Mat cam2pts(2, total, CV_64FC1);   // 图像1 中的 2D 点
    cv::Mat P1(3, 4, CV_64FC1), P2(3, 4, CV_64FC1);   // 相机投影矩阵，包含了内参和外参

    cv::Mat cam1, cam2, distort1, distort2;
    cam1 = (cv::Mat_<double>(3,3) << fx, 0 , cx, 0, fy, cy, 0, 0, 1);
    cam2 = cam1.clone();

    cv::Mat R_T_1 = (cv::Mat_<double>(3, 4) <<
                     R_1w(0,0), R_1w(0,1), R_1w(0,2), t_1w[0],
                     R_1w(1,0), R_1w(1,1), R_1w(1,2), t_1w[1],
                     R_1w(2,0), R_1w(2,1), R_1w(2,2), t_1w[2]);

    cv::Mat R_T_2 = (cv::Mat_<double>(3, 4) <<
                  R_2w(0,0), R_2w(0,1), R_2w(0,2), t_2w[0],
                  R_2w(1,0), R_2w(1,1), R_2w(1,2), t_2w[1],
                  R_2w(2,0), R_2w(2,1), R_2w(2,2), t_2w[2]);


    P1 = cam1 * R_T_1;
    P2 = cam2 * R_T_2;

    double *data1x, *data1y, *data2x, *data2y;
    data1x = cam1pts.ptr<double>(0);
    data1y = cam1pts.ptr<double>(1);
    data2x = cam2pts.ptr<double>(0);
    data2y = cam2pts.ptr<double>(1);
    for(int i=0; i<total; i++)
    {

        data1x[i] = pts1[i].x;
        data1y[i] = pts1[i].y;
        data2x[i] = pts2[i].x;
        data2y[i] = pts2[i].y;
    }

    cv::triangulatePoints(P1, P2, cam1pts, cam2pts, pts3D);

    // 复制数据
    cloud_xyz = cv::Mat(total, 3, CV_32FC1);
    cloud_rgb = cv::Mat(total, 3, CV_8UC1);

    data1x = cam1pts.ptr<double>(0);
    data1y = cam1pts.ptr<double>(1);
    uchar *data_rgb;
    for(int i=0; i<total; i++)
    {
        if(status[i])
        {
            data_rgb = cloud_rgb.ptr<uchar>(i);
            int x = data1x[i];
            int y = data1y[i];
            cv::Vec3b color = ref_image_color.at<cv::Vec3b>(y, x);
            data_rgb[0] = color[2];
            data_rgb[1] = color[1];
            data_rgb[2] = color[0];
        }
    }

    double *data_x, *data_y, *data_z, *data_w;
    float *data_cloud_xyz;
    data_x = pts3D.ptr<double>(0);
    data_y = pts3D.ptr<double>(1);
    data_z = pts3D.ptr<double>(2);
    data_w = pts3D.ptr<double>(3);
    for(int i=0; i<total; i++)
    {
        if(status[i])
        {
            data_cloud_xyz = cloud_xyz.ptr<float>(i);
            double x = data_x[i];
            double y = data_y[i];
            double z = data_z[i];
            double w = data_w[i];
            data_cloud_xyz[0] = x/w;
            data_cloud_xyz[1] = y/w;
            data_cloud_xyz[2] = z/w;
        }
    }
}


inline
void TransformLocalCloud2GlobalCloud(cv::Mat& cloud_xyz_global, cv::Mat &cloud_rgb_global,
                                     cv::Mat& cloud_xyz_local, cv::Mat &cloud_rgb_local,
                                     Eigen::Matrix3f& R_wc, Eigen::Vector3f& t_wc)
{
//    cv::Mat cloud_xyz = cloud_xyz_local.t();
    cv::Mat cloud_xyz_homo = cv::Mat::ones(4, cloud_xyz_local.rows, CV_32FC1);
    cloud_xyz_homo.rowRange(0, 3) = cloud_xyz_local.t();
    cv::Mat T_wc = (cv::Mat_<float>(3, 4) <<
                    R_wc(0,0), R_wc(0,1), R_wc(0,2), t_wc[0],
                    R_wc(1,0), R_wc(1,1), R_wc(1,2), t_wc[1],
                    R_wc(2,0), R_wc(2,1), R_wc(2,2), t_wc[2]
            );
    cloud_xyz_global = T_wc * cloud_xyz_homo;
    cloud_xyz_global = cloud_xyz_global.t();
}


// 关于去掉孤立小区域的代码
typedef cv::Point_<unsigned short> Point2s;


template <typename T>
inline
void FilterSpecklesImpl(cv::Mat& img, T newVal, int maxSpeckleSize, T maxDiff, cv::Mat& _buf)
{
    using namespace cv;

    int width = img.cols, height = img.rows, npixels = width*height;
    size_t bufSize = npixels*(int)(sizeof(Point2s) + sizeof(int) + sizeof(uchar));
    if( !_buf.isContinuous() || _buf.empty() || _buf.cols*_buf.rows*_buf.elemSize() < bufSize )
        _buf.reserveBuffer(bufSize);

    uchar* buf = _buf.ptr();
    int i, j, dstep = (int)(img.step/sizeof(T));
    int* labels = (int*)buf;
    buf += npixels*sizeof(labels[0]);
    Point2s* wbuf = (Point2s*)buf;
    buf += npixels*sizeof(wbuf[0]);
    uchar* rtype = (uchar*)buf;
    int curlabel = 0;

    // clear out label assignments
    memset(labels, 0, npixels*sizeof(labels[0]));

    for( i = 0; i < height; i++ )
    {
        T* ds = img.ptr<T>(i); // 第 i 行的地址
        int* ls = labels + width*i;  // 标签， 初始标签全是 0

        for( j = 0; j < width; j++ )
        {
            if( ds[j] != newVal )   // not a bad disparity
            {
                if( ls[j] )     // has a label, check for bad label
                {
                    if( rtype[ls[j]] ) // small region, zero out disparity
                        ds[j] = (T)newVal;
                }
                // no label, assign and propagate
                else
                {
                    Point2s* ws = wbuf; // initialize wavefront
                    Point2s p((short)j, (short)i);  // current pixel
                    curlabel++; // next label
                    int count = 0;  // current region size
                    ls[j] = curlabel;

                    // wavefront propagation
                    while( ws >= wbuf ) // wavefront not empty
                    {
                        count++;
                        // put neighbors onto wavefront
                        T* dpp = &img.at<T>(p.y, p.x);
                        T dp = *dpp;
                        int* lpp = labels + width*p.y + p.x;

                        if( p.y < height-1 && !lpp[+width] && dpp[+dstep] != newVal && std::abs(dp - dpp[+dstep]) <= maxDiff )
                        {
                            lpp[+width] = curlabel;
                            *ws++ = Point2s(p.x, p.y+1);
                        }

                        if( p.y > 0 && !lpp[-width] && dpp[-dstep] != newVal && std::abs(dp - dpp[-dstep]) <= maxDiff )
                        {
                            lpp[-width] = curlabel;
                            *ws++ = Point2s(p.x, p.y-1);
                        }

                        if( p.x < width-1 && !lpp[+1] && dpp[+1] != newVal && std::abs(dp - dpp[+1]) <= maxDiff )
                        {
                            lpp[+1] = curlabel;
                            *ws++ = Point2s(p.x+1, p.y);
                        }

                        if( p.x > 0 && !lpp[-1] && dpp[-1] != newVal && std::abs(dp - dpp[-1]) <= maxDiff )
                        {
                            lpp[-1] = curlabel;
                            *ws++ = Point2s(p.x-1, p.y);
                        }

                        // pop most recent and propagate
                        // NB: could try least recent, maybe better convergence
                        p = *--ws;
                    }

                    // assign label type
                    if( count <= maxSpeckleSize )   // speckle region
                    {
                        rtype[ls[j]] = 1;   // small region label
                        ds[j] = (T)newVal;
                    }
                    else
                        rtype[ls[j]] = 0;   // large region label
                }
            }
        }
    }
}


inline
void ComputeProjectMatrix(Eigen::Matrix<float, 3, 4>& P, ImagePoseStruct pose, float fx, float fy, float cx, float cy)
{
    Eigen::Matrix3f K;
    K << fx, 0, cx, 0, fy, cy, 0, 0, 1;
    Eigen::Matrix3f R_wc = pose.R_;
    Eigen::Vector3f t_wc = pose.t_;
    P.leftCols<3>() = R_wc.transpose();
    P.rightCols<1>() = -R_wc.transpose() * t_wc;
    P = K *P;
}


inline
void TriangulateMultiviewOnePoint(cv::Point3f& pt3, vector<cv::Point2f>& pts2, vector<Eigen::Matrix<float, 3, 4>>& project_matries)
{
    int num_of_pose = project_matries.size();
    Eigen::MatrixXf svd_A(2 * num_of_pose, 4);
    int svd_idx = 0;

    for(int i=0; i<num_of_pose; i++)
    {
        Eigen::Matrix<float, 3, 4> P = project_matries[i];
        svd_A.row(svd_idx++) = pts2[i].x * P.row(2) - P.row(0);
        svd_A.row(svd_idx++) = pts2[i].y * P.row(2) - P.row(1);
    }

    Eigen::Vector4f svd_V = Eigen::JacobiSVD<Eigen::MatrixXf>(svd_A, Eigen::ComputeThinV).matrixV().rightCols<1>();

   pt3.x = svd_V[0] / svd_V[3];
   pt3.y = svd_V[1] / svd_V[3];
   pt3.z = svd_V[2] / svd_V[3];
}


inline
void TriangulateMultiview(vector<cv::Point3f>& pts3, vector<vector<cv::Point2f>>& pts2, vector<ImagePoseStruct>& poses,
                          float fx, float fy, float cx, float cy)
{
    // 计算投影矩阵
    Eigen::Matrix<float, 3, 4> P;
    vector<Eigen::Matrix<float, 3, 4>> project_matries;
    for(int i=0; i<poses.size(); i++)
    {
        ComputeProjectMatrix(P, poses[i], fx, fy, cx, cy);
        project_matries.push_back(P);
    }

    cv::Point3f pt3;
    for(int i=0; i<pts2.size(); i++)
    {
        TriangulateMultiviewOnePoint(pt3, pts2[i], project_matries);
        pts3[i] = pt3;
    }

}


inline
void TriangulateMultiview(vector<cv::Point3f>& pts3, vector<vector<cv::Point2f>>& pts2, vector<ImagePoseStruct>& poses,
                          float fx, float fy, float cx, float cy, int pts2_num)
{
    // 计算投影矩阵
    Eigen::Matrix<float, 3, 4> P;
    vector<Eigen::Matrix<float, 3, 4>> project_matries;
    for(int i=0; i<poses.size(); i++)
    {
        ComputeProjectMatrix(P, poses[i], fx, fy, cx, cy);
        project_matries.push_back(P);
    }

    cv::Point3f pt3;
    for(int i=0; i<pts2_num; i++)
    {
        TriangulateMultiviewOnePoint(pt3, pts2[i], project_matries);
        pts3[i] = pt3;
    }

}


inline
void TriangulateMultiviewOnePoint(Eigen::Vector3f& pt3, vector<Eigen::Vector2f>& pts2, vector<Eigen::Matrix<float, 3, 4>>& project_matries)
{
    int num_of_pose = project_matries.size();
    Eigen::MatrixXf svd_A(2 * num_of_pose, 4);
    int svd_idx = 0;
    
    for(int i=0; i<num_of_pose; i++)
    {
        Eigen::Matrix<float, 3, 4> P = project_matries[i];
        svd_A.row(svd_idx++) = pts2[i][0] * P.row(2) - P.row(0);
        svd_A.row(svd_idx++) = pts2[i][1] * P.row(2) - P.row(1);
    }
    
    Eigen::Vector4f svd_V = Eigen::JacobiSVD<Eigen::MatrixXf>(svd_A, Eigen::ComputeThinV).matrixV().rightCols<1>();
    
    pt3[0] = svd_V[0] / svd_V[3];
    pt3[1] = svd_V[1] / svd_V[3];
    pt3[2] = svd_V[2] / svd_V[3];
}


inline
void TriangulateMultiview(vector<Eigen::Vector3f>& pts3, vector<vector<Eigen::Vector2f>>& pts2, vector<ImagePoseStruct>& poses,
                          float fx, float fy, float cx, float cy, int pts2_num)
{
    // 计算投影矩阵
    Eigen::Matrix<float, 3, 4> P;
    vector<Eigen::Matrix<float, 3, 4>> project_matries;
    for(int i=0; i<poses.size(); i++)
    {
        ComputeProjectMatrix(P, poses[i], fx, fy, cx, cy);
        project_matries.push_back(P);
    }
    
    Eigen::Vector3f pt3;
    for(int i=0; i<pts2_num; i++)
    {
        TriangulateMultiviewOnePoint(pt3, pts2[i], project_matries);
        pts3[i] = pt3;
    }
    
}


inline
void TriangulateTwoview(vector<Eigen::Vector3f>& pts3, vector<vector<Eigen::Vector2f>>& pts2, vector<ImagePoseStruct>& poses,
                          float fx, float fy, float cx, float cy, int pts2_num)
{
    int total = pts2_num;
    cv::Mat pts3D(4, total, CV_64FC1);  // 3D 点
    cv::Mat cam1pts(2, total, CV_64FC1);   // 图像0 中的 2D 点
    cv::Mat cam2pts(2, total, CV_64FC1);   // 图像1 中的 2D 点
    cv::Mat P1(3, 4, CV_64FC1), P2(3, 4, CV_64FC1);   // 相机投影矩阵，包含了内参和外参

    Eigen::Matrix<float, 3, 4> P1_eigen, P2_eigen;
    ComputeProjectMatrix(P1_eigen, poses[0], fx, fy, cx, cy);
    ComputeProjectMatrix(P2_eigen, poses[1], fx, fy, cx, cy);
    P1 = (cv::Mat_<double>(3, 4) <<
          P1_eigen(0, 0), P1_eigen(0, 1), P1_eigen(0, 2), P1_eigen(0, 3),
          P1_eigen(1, 0), P1_eigen(1, 1), P1_eigen(1, 2), P1_eigen(1, 3),
          P1_eigen(2, 0), P1_eigen(2, 1), P1_eigen(2, 2), P1_eigen(2, 3));
    P2 = (cv::Mat_<double>(3, 4) <<
          P2_eigen(0, 0), P2_eigen(0, 1), P2_eigen(0, 2), P2_eigen(0, 3),
          P2_eigen(1, 0), P2_eigen(1, 1), P2_eigen(1, 2), P2_eigen(1, 3),
          P2_eigen(2, 0), P2_eigen(2, 1), P2_eigen(2, 2), P2_eigen(2, 3));

    double *data1x, *data1y, *data2x, *data2y;
    data1x = cam1pts.ptr<double>(0);
    data1y = cam1pts.ptr<double>(1);
    data2x = cam2pts.ptr<double>(0);
    data2y = cam2pts.ptr<double>(1);
    for(int i=0; i<total; i++)
    {

        data1x[i] = pts2[i][0][0];
        data1y[i] = pts2[i][0][1];
        data2x[i] = pts2[i][1][0];
        data2y[i] = pts2[i][1][1];
    }

    cv::triangulatePoints(P1, P2, cam1pts, cam2pts, pts3D);

    // 复制数据
    double *data_x, *data_y, *data_z, *data_w;
    data_x = pts3D.ptr<double>(0);
    data_y = pts3D.ptr<double>(1);
    data_z = pts3D.ptr<double>(2);
    data_w = pts3D.ptr<double>(3);
    Eigen::Vector3f pt3;
    for(int i=0; i<total; i++)
    {
        double x = data_x[i];
        double y = data_y[i];
        double z = data_z[i];
        double w = data_w[i];
        pt3[0] = x/w;
        pt3[1] = y/w;
        pt3[2] = z/w;
        pts3[i] = pt3;
    }
}


inline
bool SavePlyFile(string filename, cv::Mat& cloud_xyz, cv::Mat& cloud_rgb)
{
    int point_num = cloud_xyz.rows;

    FILE *fp = NULL;
    fp = fopen(filename.c_str(), "w");

    if(NULL == fp) // 判断文件打开是否成功
        return false;
    fprintf(fp, "ply\n");
    fprintf(fp, "format binary_little_endian 1.0\n");
    fprintf(fp, "element vertex %d\n", point_num);
    fprintf(fp, "property float x\n");
    fprintf(fp, "property float y\n");
    fprintf(fp, "property float z\n");
    fprintf(fp, "property uchar red\n");
    fprintf(fp, "property uchar green\n");
    fprintf(fp, "property uchar blue\n");
    fprintf(fp, "property uchar alpha\n");
    fprintf(fp, "end_header\n");

    // Create point cloud content for ply file
    for (int i = 0; i < point_num; i++) {
        // Convert voxel indices to float, and save coordinates to ply file
        float pt_base_x = cloud_xyz.at<float>(i, 0);
        float pt_base_y = cloud_xyz.at<float>(i, 1);
        float pt_base_z = cloud_xyz.at<float>(i, 2);

        if(!(pt_base_x < 10 && pt_base_x > -10 &&
                pt_base_y < 10 && pt_base_y > -10 &&
                pt_base_z < 10 && pt_base_z > -10))
        {
            pt_base_x = pt_base_y = pt_base_z = 0.0f;
        }

        uchar color_r = cloud_rgb.at<uchar>(i, 0);
        uchar color_g = cloud_rgb.at<uchar>(i, 1);
        uchar color_b = cloud_rgb.at<uchar>(i, 2);
        uchar color_a = 255;

        fwrite(&pt_base_x, sizeof(float), 1, fp);
        fwrite(&pt_base_y, sizeof(float), 1, fp);
        fwrite(&pt_base_z, sizeof(float), 1, fp);
        fwrite(&color_r, sizeof(uchar), 1, fp);
        fwrite(&color_g, sizeof(uchar), 1, fp);
        fwrite(&color_b, sizeof(uchar), 1, fp);
        fwrite(&color_a, sizeof(uchar), 1, fp);
        }
    fclose(fp);

    return true;
}


inline
void TransformCloudByMatrix(cv::Mat& cloud_xyz_transformed,
                                     cv::Mat& cloud_xyz,
                                     ImagePoseStruct T)
{

    cv::Mat cloud_xyz_homo = cv::Mat::ones(4, cloud_xyz.cols, CV_32FC1);
    cloud_xyz.copyTo(cloud_xyz_homo.rowRange(0,3));
    cv::Mat T_mat = (cv::Mat_<float>(3, 4) <<
                    T.R_(0,0), T.R_(0,1), T.R_(0,2), T.t_[0],
                    T.R_(1,0), T.R_(1,1), T.R_(1,2), T.t_[1],
                    T.R_(2,0), T.R_(2,1), T.R_(2,2), T.t_[2]
            );
    cloud_xyz_transformed = T_mat * cloud_xyz_homo;
}


inline
void TransformCloudByMatrix(vector<Eigen::Vector3f>& pts,
                            const vector<cv::Point3f> ptsOrigin,
                            const ImagePoseStruct& Twc,
                            int pointNum=0)
{
    if(pointNum == 0)
        pointNum = ptsOrigin.size();
    
    if(pts.size() < pointNum)
        pts.resize(pointNum);

    // 第一步，Twc 转成 Tcw
    ImagePoseStruct Tcw;
    Tcw.R_ = Twc.R_.transpose();
    Tcw.t_ = -Twc.R_.transpose() * Twc.t_;

    // 第二步，将世界坐标系下的点云转到局部坐标系
    // 先转换数据类型
    Eigen::MatrixXf cloudGlobal(3, pointNum);
    for(int idx=0; idx<pointNum; idx++)
    {
        cloudGlobal(0, idx) = ptsOrigin[idx].x;
        cloudGlobal(1, idx) = ptsOrigin[idx].y;
        cloudGlobal(2, idx) = ptsOrigin[idx].z;
    }

    Eigen::MatrixXf cloudTransformed;
    cloudTransformed = Tcw.R_ * cloudGlobal;

    Eigen::Vector3f pt;
    float x = Tcw.t_[0];
    float y = Tcw.t_[1];
    float z = Tcw.t_[2];
    for(int idx=0; idx<pointNum; idx++)
    {
        pt[0] = cloudTransformed(0, idx) + x;
        pt[1] = cloudTransformed(1, idx) + y;
        pt[2] = cloudTransformed(2, idx) + z;
        pts[idx] = pt;
    }
}


inline
void TransformCloudByMatrix(vector<Eigen::Vector3f>& pts,
                            const vector<Eigen::Vector3f> ptsOrigin,
                            const ImagePoseStruct& Twc,
                            int pointNum=0)
{
    if(pointNum == 0)
        pointNum = ptsOrigin.size();
    
    if(pts.size() < pointNum)
        pts.resize(pointNum);
    
    // 第一步，Twc 转成 Tcw
    ImagePoseStruct Tcw;
    Tcw.R_ = Twc.R_.transpose();
    Tcw.t_ = -Twc.R_.transpose() * Twc.t_;
    
    // 第二步，将世界坐标系下的点云转到局部坐标系
    // 先转换数据类型
    Eigen::MatrixXf cloudGlobal(3, pointNum);
    for(int idx=0; idx<pointNum; idx++)
    {
        cloudGlobal(0, idx) = ptsOrigin[idx][0];
        cloudGlobal(1, idx) = ptsOrigin[idx][1];
        cloudGlobal(2, idx) = ptsOrigin[idx][2];
    }
    
    Eigen::MatrixXf cloudTransformed;
    cloudTransformed = Tcw.R_ * cloudGlobal;
    
    Eigen::Vector3f pt;
    float x = Tcw.t_[0];
    float y = Tcw.t_[1];
    float z = Tcw.t_[2];
    for(int idx=0; idx<pointNum; idx++)
    {
        pt[0] = cloudTransformed(0, idx) + x;
        pt[1] = cloudTransformed(1, idx) + y;
        pt[2] = cloudTransformed(2, idx) + z;
        pts[idx] = pt;
    }
}


inline
void RelativePose(ImagePoseStruct& T_ref_tgt, const ImagePoseStruct& refTwc, const ImagePoseStruct& tgtTwc)
{
    T_ref_tgt.R_ = refTwc.R_.transpose() * tgtTwc.R_;
    T_ref_tgt.t_ = refTwc.R_.transpose() * (tgtTwc.t_ - refTwc.t_);
    T_ref_tgt.image_index_ = tgtTwc.image_index_;
}


// 计算 pose 差异
inline
float ComputePoseDifference(ImagePoseStruct& pose1, ImagePoseStruct& pose2,
                            float desiredDdeltB=0.1, float desiredDeltZRatio=0.2, float desiredDeltTheta=2.f)
{
    float deltB=0; // 基线 （ xy平面上的位移 )
    float deltZ = 0; // z 方向上的位移
    float deltTheta = 0; // 角度差异

    ImagePoseStruct relativePose;
    RelativePose(relativePose, pose1, pose2);

    float x = relativePose.t_[0];
    float y = relativePose.t_[1];
    float z = relativePose.t_[2];
    deltB = sqrt(x*x + y*y);
    deltZ = fabs(z);

    Eigen::Vector3f e1 = relativePose.R_.col(2); // z方向
    Eigen::Vector3f e2(0-x, 0-y, 1.2f-z);  // 从第二个相机中心到第一个相机前方 2 米处的一个点 这个方向
    e2.normalize();
    Eigen::VectorXf temp = e1.transpose() * e2;
    deltTheta = std::acos(temp[0]) * 180 / 3.14f;

    float w1(0.2f), w2(0.1f), w3(0.1f);
    float costB = w1 * exp(fabs(deltB - desiredDdeltB)/0.01f);  // 基线越接近 0.1 米越好
    float costZ = w2 * exp(deltZ/deltB/desiredDeltZRatio); // z 的变化相比基线的长度，越小越好，最好小于 0.1
    float costTheta = w3 * exp(deltTheta/desiredDeltTheta);  // 角度差异越小越好，最好是小于 pi/10;
    float cost = costB + costZ + costTheta;

    // 第三计算 cost
    return cost;
}


// 寻找最合适的一帧做匹配，返回值是 cost
inline
float FindBestPose(ImagePoseStruct& bestPose, ImagePoseStruct& refPose, list<ImagePoseStruct>& poseBuffer,
                 float desiredDdeltB=0.1f, float desiredDeltZRatio=0.2f, float desiredDeltTheta=2)
{
    // 从 buffer 里面找出最好的一个
    float costBest = 9999;
    auto itBest = poseBuffer.begin();
    auto itEnd = poseBuffer.end();
    itEnd--;  // 最后一个就是自身，应该被排除在外
    for(auto it = poseBuffer.begin(); it != itEnd; ++it)
    {
        float costCurr = ComputePoseDifference(refPose, *it, desiredDdeltB, desiredDeltZRatio, desiredDeltTheta);
        if(costCurr < costBest)
        {
            costBest = costCurr;
            itBest = it;
        }
    }

    // 找到了合适的，提取出来
    bestPose = *itBest;

    // ---------------- debug -------------------
//    float deltB=0; // 基线 （ xy平面上的位移 )
//    float deltZ = 0; // z 方向上的位移
//    float deltTheta = 0; // 角度差异
//
//    ImagePoseStruct relativePose;
//    RelativePose(relativePose, refPose, bestPose);
//
//    float x = relativePose.t_[0];
//    float y = relativePose.t_[1];
//    float z = relativePose.t_[2];
//    deltB = sqrt(x*x + y*y);
//    deltZ = fabs(z);
//
//    Eigen::Vector3f e1 = relativePose.R_.col(2); // z方向
//    Eigen::Vector3f e2(0-x, 0-y, 2-z);  // 从第二个相机中心到第一个相机前方 2 米处的一个点 这个方向
//    e2.normalize();
//    Eigen::VectorXf temp = e1.transpose() * e2;
//    deltTheta = std::acos(temp[0]) * 180 / 3.14f;
//
//    cout << "deltB: " << deltB << "    deltZ: " << deltZ << "    deltTheta: " << deltTheta << endl;

    return costBest;
}


// 判断一张图片是否是模糊的，依据是看与前几帧相比梯度总量是否有明显下降
inline
bool IsBlurred(cv::Mat& img)
{
    static std::list<float> historyDefinition;
    static float maxDefinition;

    cv::Scalar s;
    float definition;
    cv::Mat A;
    img.convertTo(A, CV_16S);
    cv::Mat blurredA, diff;
    cv::boxFilter(A, blurredA, A.depth(), cv::Size(3, 3));
    cv::absdiff(A, blurredA, diff);
    s = cv::sum(diff);
    definition = s[0];
    maxDefinition = definition;

    if(historyDefinition.size() < 5)
    {
        historyDefinition.emplace_back(definition);
        return true;  // 如果还没有数据，那就返回是模糊的
    }
    else
    {
        // 事先插入可以保证后面的 down是大于等于0 的
        historyDefinition.pop_front();
        historyDefinition.emplace_back(definition);

        for(auto d : historyDefinition)  // 找出前几帧里面清晰度最高的
        {
            if(d > maxDefinition)
                maxDefinition = d;
        }

        // 看清晰度是否有明显下降
        float down = (float)(maxDefinition - definition) / maxDefinition;
        if(down > 0.3f)
            return true;
        else
            return false;
    }
}


inline
bool IsSubKeyFrame(const ImagePoseStruct& poseLast, const ImagePoseStruct& poseCurr,
                float B_thresh, float Z_thresh, float theta_thresh)
{
    float deltB=0; // 基线 （ xy平面上的位移 )
    float deltZ = 0; // z 方向上的位移
    float deltTheta = 0; // 角度差异

    ImagePoseStruct relativePose;
    RelativePose(relativePose, poseCurr, poseLast);

    float temp1 = relativePose.t_[0];
    float temp2 = relativePose.t_[1];
    deltB = sqrt(temp1*temp1 + temp2*temp2);
    deltZ = fabs(relativePose.t_[2]);

    Eigen::Matrix3f delt_R = relativePose.R_;
    Eigen::AngleAxisf delt_r;
    delt_r.fromRotationMatrix(delt_R);
    deltTheta = fabs(delt_r.angle());

    // xyz方向上位移足够大或者角度变化足够大时就可以作为subkeyframe了
    if(deltB < B_thresh && deltZ < Z_thresh && deltTheta < theta_thresh*3.14f/180)
        return false;

    return true;
}


inline
bool IsKeyFrame(const ImagePoseStruct& poseLast, const ImagePoseStruct& poseCurr,
                float B_thresh, float Z_thresh, float theta_thresh)
{
    return IsSubKeyFrame(poseLast, poseCurr, B_thresh, Z_thresh, theta_thresh);
}


inline
void FilterOutlierByFMatrix(cv::Mat& optflow, cv::Mat& validMask, const ImagePoseStruct& refTwc, const ImagePoseStruct& tgtTwc,
                            float fx, float fy, float cx, float cy)
{
    // ---------------- 计算基础矩阵 F ----------------------
    Eigen::Matrix3f F; // 基础矩阵
    // 相对旋转和平移, ref 是1, tgt是2
    Eigen::Matrix3f R21 = tgtTwc.R_.transpose() * refTwc.R_;
    Eigen::Vector3f t21 = tgtTwc.R_.transpose() * (refTwc.t_ - tgtTwc.t_);
    Eigen::Matrix3f t21_hat; // 用于叉乘
    t21_hat <<
            0, -t21[2], t21[1],
            t21[2], 0, -t21[0],
            -t21[1], t21[0], 0;

    Eigen::Matrix3f K; // 内参矩阵
    K << fx, 0, cx, 0, fy, cy, 0, 0, 1;

    Eigen::Matrix3f K_inv = K.inverse();
    F = K_inv.transpose() * t21_hat * R21 * K_inv;

    // extract fundamental matrix
    float f00 = F(0, 0); float f01 = F(0, 1); float f02 = F(0, 2);
    float f10 = F(1, 0); float f11 = F(1, 1); float f12 = F(1, 2);
    float f20 = F(2, 0); float f21 = F(2, 1); float f22 = F(2, 2);

    // loop variables
    float u1,v1,u2,v2;
    float x2tFx1;
    float Fx1u,Fx1v,Fx1w;
    float Ftx2u,Ftx2v;

    float inlier_threshold = 1.0f * optflow.cols/640.f;  // 在640*480尺寸的时候，设置阈值为1是比较好的，那么图片如果缩小了，应该也要把阈值减小一点比较好
    for(int row = 0; row < optflow.rows; row++)
    {
        cv::Vec2f* ptr_optflow = optflow.ptr<cv::Vec2f>(row);
        uchar* ptrValidMask = validMask.ptr<uchar>(row);
        for(int col=0; col<optflow.cols; col++)
        {
            cv::Vec2f temp = ptr_optflow[col];
            u1 = col;
            v1 = row;
            u2 = col + temp[0];
            v2 = row + temp[1];

            // F*x1
            Fx1u = f00*u1+f01*v1+f02;
            Fx1v = f10*u1+f11*v1+f12;
            Fx1w = f20*u1+f21*v1+f22;

            // F'*x2
            Ftx2u = f00*u2+f10*v2+f20;
            Ftx2v = f01*u2+f11*v2+f21;

            // x2'*F*x1
            x2tFx1 = u2*Fx1u+v2*Fx1v+Fx1w;

            // sampson distance
            float d = x2tFx1*x2tFx1 / (Fx1u*Fx1u+Fx1v*Fx1v+Ftx2u*Ftx2u+Ftx2v*Ftx2v);

            if(fabs(d)<inlier_threshold)
            {
                ptrValidMask[col] = 1;
            }
        }
    }
}


// 进来的是世界坐标系下的点云
inline
void Points2Depthmap(cv::Mat& depthmap, int& pointsNumRemain, const vector<cv::Point3f>& pts3, const vector<vector<cv::Point2f>>& ptsVec, int ptsVecSize,
                     const ImagePoseStruct& Twc, float fx, float fy, float cx, float cy, int downsample)
{
    // 第一步，Twc 转成 Tcw
    ImagePoseStruct Tcw;
    Tcw.R_ = Twc.R_.transpose();
    Tcw.t_ = -Twc.R_.transpose() * Twc.t_;

    // 第二步，将世界坐标系下的点云转到局部坐标系
    // 先转换数据类型
    cv::Mat cloud_global(3, ptsVecSize, CV_32F);
    for(int idx=0; idx<ptsVecSize; idx++)
    {
        cv::Point3f pt = pts3[idx];
        cloud_global.ptr<float>(0)[idx] = pt.x;
        cloud_global.ptr<float>(1)[idx] = pt.y;
        cloud_global.ptr<float>(2)[idx] = pt.z;
    }
    // 再转换坐标系
    cv::Mat cloud_local(3, ptsVecSize, CV_32F);
    TransformCloudByMatrix(cloud_local, cloud_global, Tcw);

    pointsNumRemain = 0;
    int origWidth = depthmap.cols*downsample;
    int origHeight = depthmap.rows*downsample;
    for(int idx=0; idx<ptsVecSize; idx++)
    {
        cv::Point2f pt = ptsVec[idx][0];
        float u = pt.x;
        float v = pt.y;
        float x, y, z;
        x = cloud_local.ptr<float>(0)[idx];
        y = cloud_local.ptr<float>(1)[idx];
        z = cloud_local.ptr<float>(2)[idx];
        if(z < 0.1f || z > 5.0f) // 深度值太大或者太小肯定都是噪声
            continue;
        float u2 = x / z * fx + cx;
        float v2 = y / z * fy + cy;
        if(fabs(u2 - u) > 2 || fabs(v2 - v) > 2 || u2 < 0 || u2 >= origWidth || v2 < 0 || v2 >= origHeight)
            continue;
        depthmap.ptr<float>(int(v2/downsample))[int(u2/downsample)] = z;
        pointsNumRemain++;
    }
}


// 进来的直接就是局部点云
inline
void Points2Depthmap(cv::Mat& depthmap, int& pointsNumRemain, const vector<Eigen::Vector3f>& pts3, const vector<vector<cv::Point2f>>& ptsVec, int ptsVecSize, float fx, float fy, float cx, float cy, int downsample, float nearPlane, float farPlane, float projectionErrorTolerance)
{
    pointsNumRemain = 0;
    int origWidth = depthmap.cols*downsample;
    int origHeight = depthmap.rows*downsample;
    for(int idx=0; idx<ptsVecSize; idx++)
    {
        cv::Point2f pt = ptsVec[idx][0];
        float u = pt.x;
        float v = pt.y;
        float x, y, z;
        x = pts3[idx][0];
        y = pts3[idx][1];
        z = pts3[idx][2];
        if(z < nearPlane || z > farPlane) // 深度值太大或者太小肯定都是噪声
            continue;
        float u2 = x / z * fx + cx;
        float v2 = y / z * fy + cy;
        if(fabs(u2 - u) > projectionErrorTolerance || fabs(v2 - v) > projectionErrorTolerance || u2 < 0 || u2 >= origWidth || v2 < 0 || v2 >= origHeight)
            continue;
        depthmap.ptr<float>(int(v2/downsample))[int(u2/downsample)] = z;
        pointsNumRemain++;
    }
}


// 进来的直接就是局部点云
inline
void Points2Depthmap(cv::Mat& depthmap, int& pointsNumRemain, const vector<Eigen::Vector3f>& pts3, const vector<vector<Eigen::Vector2f>>& ptsVecVec, int ptsVecSize, float fx, float fy, float cx, float cy, int downsample, float nearPlane, float farPlane, float projectionErrorTolerance)
{
    pointsNumRemain = 0;
    int origWidth = depthmap.cols*downsample;
    int origHeight = depthmap.rows*downsample;
    for(int idx=0; idx<ptsVecSize; idx++)
    {
        Eigen::Vector2f pt = ptsVecVec[idx][0];
        float u = pt[0];
        float v = pt[1];
        float x, y, z;
        x = pts3[idx][0];
        y = pts3[idx][1];
        z = pts3[idx][2];
        if(z < nearPlane || z > farPlane) // 深度值太大或者太小肯定都是噪声
            continue;
        float u2 = x / z * fx + cx;
        float v2 = y / z * fy + cy;
        if(fabs(u2 - u) > projectionErrorTolerance || fabs(v2 - v) > projectionErrorTolerance || u2 < 0 || u2 >= origWidth || v2 < 0 || v2 >= origHeight)
            continue;
        depthmap.ptr<float>(int(v2/downsample))[int(u2/downsample)] = z;
        pointsNumRemain++;
    }
}


// 进来的直接就是局部点云
inline
void Points2Depthmap(cv::Mat& depthmap, int& pointsNumRemain, const vector<Eigen::Vector3f>& pts3, int ptsVecSize, float fx, float fy, float cx, float cy, int downsample, float nearPlane, float farPlane)
{
    pointsNumRemain = 0;
    fx /= downsample;
    fy /= downsample;
    cx /= downsample;
    cy /= downsample;
    for(int idx=0; idx<ptsVecSize; idx++)
    {
        float x, y, z;
        x = pts3[idx][0];
        y = pts3[idx][1];
        z = pts3[idx][2];
        if(z < nearPlane || z > farPlane) // 深度值太大或者太小肯定都是噪声
            continue;
        int u2 = x / z * fx + cx;
        int v2 = y / z * fy + cy;
        if(u2 > 0 && u2 < depthmap.cols -1 && v2 > 0 && v2 < depthmap.rows-1)
        {
            depthmap.ptr<float>(v2)[u2] = z;
            pointsNumRemain++;
        }
    }
}

#endif //HELPER_FUNCTIONS_H
