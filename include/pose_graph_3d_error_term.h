#ifndef POSE_GRAPH_3D_ERROR_TERM_H
#define POSE_GRAPH_3D_ERROR_TERM_H

#include <Eigen/Core>
#include <ceres/autodiff_cost_function.h>

#include "types.h"

namespace ceres {

class PoseGraph3dErrorTerm
{
public:
    PoseGraph3dErrorTerm(const Pose3d& t_ab_measured,
                         const Eigen::Matrix<double, 6, 6>& sqrt_information)
        :t_ab_measured_(t_ab_measured), sqrt_information_(sqrt_information)
    {}

    template <typename T>
    bool operator ()(const T* const p_a_ptr, const T* const q_a_ptr,
                     const T* const p_b_ptr, const T* const q_b_ptr,
                     T* residuals_ptr) const
    {
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_a(p_a_ptr);
        Eigen::Map<const Eigen::Quaternion<T>>   q_a(q_a_ptr);

        Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_b(p_b_ptr);
        Eigen::Map<const Eigen::Quaternion<T>>   q_b(q_b_ptr);

        Eigen::Quaternion<T> q_a_inverse = q_a.conjugate();
        Eigen::Quaternion<T> q_ab_estimated = q_a_inverse * q_b;

        Eigen::Matrix<T, 3, 1> p_ab_estimated = q_a_inverse * (p_b - p_a);

        Eigen::Quaternion<T> delta_q =
                t_ab_measured_.q.template cast<T>() * q_ab_estimated.conjugate();

        Eigen::Map<Eigen::Matrix<T, 6, 1>> residuals(residuals_ptr);
        residuals.template block<3, 1>(0, 0) =
                p_ab_estimated - t_ab_measured_.p.template cast<T>();
        residuals.template block<3, 1>(3, 0) = T(2.0) * delta_q.vec();

        residuals.applyOnTheLeft(sqrt_information_.template cast<T>());

        return true;
    }

    static ceres::CostFunction* Create(
            const Pose3d& t_ab_measured,
            const Eigen::Matrix<double, 6, 6>& sqrt_information)
    {
        return new ceres::AutoDiffCostFunction<PoseGraph3dErrorTerm, 6, 3, 4, 3, 4>(
                    new PoseGraph3dErrorTerm(t_ab_measured, sqrt_information));
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    const Pose3d t_ab_measured_;
    const Eigen::Matrix<double, 6, 6> sqrt_information_;

};

}

#endif // POSE_GRAPH_3D_ERROR_TERM_H
