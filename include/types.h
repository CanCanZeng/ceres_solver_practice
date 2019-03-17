#ifndef TYPES_H
#define TYPES_H

#include <iostream>
#include <map>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace ceres {

struct Pose3d
{
    Eigen::Vector3d p;
    Eigen::Quaterniond q;

    static std::string name()
    {
        return "VERTEX_SE3:QUAT";
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

std::istream& operator >>(std::istream& input, Pose3d& pose)
{
    input >> pose.p.x() >> pose.p.y() >> pose.p.z() >> pose.q.x() >>
                           pose.q.y() >> pose.q.z() >> pose.q.w();
    pose.q.normalize();
    return input;
}

typedef std::map<int, Pose3d, std::less<int>, Eigen::aligned_allocator<std::pair<const int, Pose3d>>>
    MapOfPoses;

struct Constraint3d
{
    int id_begin;
    int id_end;

    Pose3d t_be;

    Eigen::Matrix<double, 6, 6> information;

    static std::string name()
    {
        return "EDGE_SE3:QUAT";
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

std::istream& operator >>(std::istream& input, Constraint3d& constraint)
{
    Pose3d& t_be = constraint.t_be;
    input >> constraint.id_begin >> constraint.id_end >> t_be;

    for(int i=0; i<6 && input.good(); ++i)
    {
        for(int j=i; j<6 && input.good(); ++j)
        {
            input >> constraint.information(i, j);
            if(i != j)
            {
                constraint.information(j, i) = constraint.information(i, j);
            }
        }
    }
    return input;
}

typedef std::vector<Constraint3d, Eigen::aligned_allocator<Constraint3d>>
    VectorOfConstraints;

}

#endif // TYPES_H
