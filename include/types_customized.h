#ifndef TYPES_CUSTOMIZED_H
#define TYPES_CUSTOMIZED_H

#include <iostream>
#include <Eigen/Core>

using namespace std;

struct Float3{
    float x;
    float y;
    float z;

    Float3(float x_, float y_, float z_)
        :x(x_), y(y_), z(z_)
    {
    }

    Float3()
        :x(0), y(0), z(0)
    {
    }
};



struct Uchar3{
    typedef unsigned char uchar;
    uchar x;
    uchar y;
    uchar z;

    Uchar3(uchar x_, uchar y_, uchar z_)
        :x(x_), y(y_), z(z_)
    {
    }

    Uchar3()
        :x(0), y(0), z(0)
    {
    }
};


struct PointXYZRGB
{
    Float3 pt_;
    Uchar3 color_;

    PointXYZRGB()
    {

    }

    PointXYZRGB(Float3 pt, Uchar3 color)
        :pt_(pt), color_(color)
    {

    }
};



struct Float4{
    float x;
    float y;
    float z;
    float w;

    Float4()
        :x(0), y(0), z(0), w(0)
    {}

    Float4(float x, float y, float z, float w)
        :x(x), y(y), z(z), w(w)
    {}

    // 拷贝构造函数
    Float4(const Float4& in)
    {
        x = in.x;
        y = in.y;
        z = in.z;
        w = in.w;
    }

    Float4& operator =(const Float4& in)
    {
        x = in.x;
        y = in.y;
        z = in.z;
        w = in.w;
        return *this;
    }
};

typedef Float4 Intrinsic;



// 用于 Pose buffer 的结构体
struct ImagePoseStruct{
    unsigned long image_index_;
    Eigen::Vector3f t_;
    Eigen::Matrix3f R_;
    ImagePoseStruct()
        :image_index_(0), t_(0,0,0), R_(Eigen::Matrix3f::Zero())
    {}

    ImagePoseStruct(int image_index, Eigen::Vector3f t, Eigen::Matrix3f R)
        :image_index_(image_index), t_(t), R_(R)
    {}

    ImagePoseStruct(int image_index, pair<Eigen::Matrix3f, Eigen::Vector3f> pose)
        :image_index_(image_index)
    {
        t_ = pose.second;
        R_ = pose.first;
    }

    // 拷贝构造函数
    ImagePoseStruct(const ImagePoseStruct& pose_in)
    {
        image_index_ = pose_in.image_index_;
        t_ = pose_in.t_;
        R_ = pose_in.R_;
    }

    ImagePoseStruct& operator =(const ImagePoseStruct& pose_in)
    {
        image_index_ = pose_in.image_index_;
        t_ = pose_in.t_;
        R_ = pose_in.R_;
        return *this;
    }
};



#endif // TYPES_CUSTOMIZED_H
