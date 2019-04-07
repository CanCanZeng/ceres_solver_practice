#ifndef SLAM_FRAME_DATABASE_H
#define SLAM_FRAME_DATABASE_H

#include <iostream>
#include <vector>

namespace SLAM
{

class Frame;

class FrameDatabase
{
public:
    ~FrameDatabase();

    void AddFrame(Frame* frame) { frames_.push_back(frame);}
//    void EraseFrame(Frame* frame);
    inline size_t size() {return frames_.size();}

    Frame* operator[](size_t i) {return frames_[i];}

    std::vector<Frame*> frames_;
};

FrameDatabase::~FrameDatabase()
{
    for(size_t i=0, iEnd=frames_.size(); i<iEnd; ++i)
    {
        if(frames_[i] != nullptr)
        {
            delete frames_[i];
            frames_[i] = nullptr;
        }
    }
}

}

#endif // SLAM_FRAME_DATABASE_H
