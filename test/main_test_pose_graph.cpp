#include <iostream>
#include <fstream>
#include <string>

#include <ceres/ceres.h>

#include "pose_graph_3d_error_term.h"
#include "types.h"
#include "read_g2o.h"

namespace ceres {

void BuildOptimizationProblem(const VectorOfConstraints& constraints,
                              MapOfPoses* poses,
                              ceres::Problem* problem)
{
    assert(poses != NULL);
    assert(problem != NULL);
    if(constraints.empty())
    {
        std::cout << "No constraints, no problem to optimize." << std::endl;
        return;
    }

    ceres::LossFunction* loss_function = NULL;
    ceres::LocalParameterization* quaternion_local_parameterization =
            new ceres::EigenQuaternionParameterization;

    for(VectorOfConstraints::const_iterator constraints_iter = constraints.begin();
        constraints_iter != constraints.end();
        ++constraints_iter)
    {
        const Constraint3d& constraint = *constraints_iter;

        MapOfPoses::iterator pose_begin_iter = poses->find(constraint.id_begin);
        CHECK(pose_begin_iter != poses->end())
                << "pose with ID: " << constraint.id_begin << " not found!";
        MapOfPoses::iterator pose_end_iter = poses->find(constraint.id_end);
        CHECK(pose_end_iter != poses->end())
                << "pose with ID: " << constraint.id_end << " not found!";

        const Eigen::Matrix<double, 6, 6> sqrt_information =
                constraint.information.llt().matrixL();

        ceres::CostFunction* cost_function = PoseGraph3dErrorTerm::Create(constraint.t_be, sqrt_information);

        problem->AddResidualBlock(cost_function, loss_function,
                                   pose_begin_iter->second.p.data(),
                                   pose_begin_iter->second.q.coeffs().data(),
                                   pose_end_iter->second.p.data(),
                                   pose_end_iter->second.q.coeffs().data());

        problem->SetParameterization(pose_begin_iter->second.q.coeffs().data(),
                                     quaternion_local_parameterization);
        problem->SetParameterization(pose_end_iter->second.q.coeffs().data(),
                                     quaternion_local_parameterization);
    }

    MapOfPoses::iterator pose_start_iter = poses->begin();
    CHECK(pose_start_iter != poses->end()) << "There are no poses.";
    problem->SetParameterBlockConstant(pose_start_iter->second.p.data());
    problem->SetParameterBlockConstant(pose_start_iter->second.q.coeffs().data());
}

bool SolveOptimizationProblem(ceres::Problem* problem)
{
    CHECK(problem != NULL);

    ceres::Solver::Options options;
    options.max_num_iterations = 200;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;

    ceres::Solver::Summary summary;
    ceres::Solve(options, problem, &summary);

    std::cout << summary.FullReport() << std::endl;
    return summary.IsSolutionUsable();
}

bool OutputPoses(const std::string& filename, const MapOfPoses& poses)
{
    std::fstream outfile;
    outfile.open(filename.c_str(), std::istream::out);
    if(!outfile)
    {
        std::cerr << "Error opening the file: " << filename;
        return false;
    }

    for(std::map<int, Pose3d, std::less<int>,
        Eigen::aligned_allocator<std::pair<const int, Pose3d>>>::const_iterator poses_iter = poses.begin();
        poses_iter != poses.end(); ++poses_iter)
    {
        const std::map<int, Pose3d, std::less<int>,
                Eigen::aligned_allocator<std::pair<const int, Pose3d>>>::
                value_type& pair = *poses_iter;
        outfile << pair.first << " " << pair.second.p.transpose() << " "
                << pair.second.q.x() << " " << pair.second.q.y() << " "
                << pair.second.q.z() << " " << pair.second.q.w() << '\n';
    }

    return true;
}

}  // name space ceres

int main(int argc, char** argv)
{
    std::string input_file(argv[1]);

    ceres::MapOfPoses poses;
    ceres::VectorOfConstraints constraints;

    CHECK(ceres::examples::ReadG2oFile(input_file, &poses, &constraints))
            << "Error reading the file: " << input_file;

    std::cout << "Number of poses: " << poses.size() << std::endl;
    std::cout << "Number of constraints: " << constraints.size() << std::endl;

    CHECK(ceres::OutputPoses("poses_original.txt", poses))
            << "Error outputing to pose_original.txt";

    ceres::Problem problem;
    ceres::BuildOptimizationProblem(constraints, &poses, &problem);

    CHECK(ceres::SolveOptimizationProblem(&problem))
            << "The solve was not successful, exiting...";

    CHECK(ceres::OutputPoses("poses_optimized.txt", poses))
            << "Error outputting to poses_original.txt";

    return 0;
}
