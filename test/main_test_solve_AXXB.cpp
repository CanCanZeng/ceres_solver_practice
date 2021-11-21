#include <iostream>

#include <ceres/ceres.h>
#include <ceres/rotation.h>

struct SO3Error {
    SO3Error(const Eigen::Quaterniond& A, const Eigen::Quaterniond& B)
        :A_(A), B_(B) {}

    template <typename T>
    bool operator()(const T* const _X,
                    T* residuals) const
    {
        Eigen::Quaternion<T> A = A_.cast<T>();
        Eigen::Quaternion<T> B = B_.cast<T>();
        Eigen::Quaternion<T> X{_X[3], _X[0], _X[1], _X[2]};

        Eigen::Quaternion<T> res = B.inverse() * X.inverse() * A * X;
        residuals[0] = res.x();
        residuals[1] = res.y();
        residuals[2] = res.z();

        return true;
    }

    static ceres::CostFunction* Create(const Eigen::Quaterniond& A, const Eigen::Quaterniond& B)
    {
        return (new ceres::AutoDiffCostFunction<SO3Error, 3, 4>(
                    new SO3Error(A, B)));
    }

    Eigen::Quaterniond A_, B_;
};


int main(int argc, char** argv)
{
    // 这个是求解机器人手眼标定问题的

    Eigen::Quaterniond X(0.9, 0.4, 0.3, 0.5);
    X.normalize();

    std::vector<Eigen::Quaterniond> A_vec;
    std::vector<Eigen::Quaterniond> B_vec;

    size_t N = 100;
    for(size_t i = 0; i < N; i++) {
        Eigen::Quaterniond A(rand() * 1.0 / RAND_MAX, rand() * 1.0 / RAND_MAX, rand() * 1.0 / RAND_MAX, rand() * 1.0 / RAND_MAX);
        A.normalize();

        Eigen::Quaterniond B = X.inverse() * A * X;
        B.normalize();

        A_vec.push_back(A);
        B_vec.push_back(B);
    }

    Eigen::Vector4d result(rand() * 1.0 / RAND_MAX, rand() * 1.0 / RAND_MAX, rand() * 1.0 / RAND_MAX, rand() * 1.0 / RAND_MAX);
    result.normalize();

    ceres::Problem problem;

    // add parameters
    ceres::LocalParameterization *q_parameterization = new ceres::QuaternionParameterization();
    problem.AddParameterBlock(result.data(), 4, q_parameterization);

    // add residuals
    for(size_t i = 0; i < A_vec.size(); ++i)
    {
        ceres::CostFunction* cost_function = SO3Error::Create(A_vec[i], B_vec[i]);
        problem.AddResidualBlock(cost_function, nullptr, result.data());
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << std::endl;

    std::cout << "X      = " << X.coeffs().transpose() << std::endl;
    std::cout << "result = " << result.transpose() << std::endl;

    return 0;
}













