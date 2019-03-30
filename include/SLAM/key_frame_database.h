#include <vector>
#include <list>
#include <set>
#include <mutex>

#include "SLAM/key_frame.h"
#include "SLAM/frame.h"

namespace SLAM
{

class KeyFrame;
class Frame;

class KeyFrameDatabase
{
public:

    void Add(KeyFrame* pKF);
    void Erase(KeyFrame* pKF);
    void Clear();

public:
    std::vector<std::list<KeyFrame*>> invertedFile_;

    std::mutex mu_;
};

}
