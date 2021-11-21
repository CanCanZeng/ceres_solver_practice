#include <iostream>

#include <ceres/ceres.h>
#include <ceres/rotation.h>

struct SO3Error {
    SO3Error(const Eigen::Quaterniond& A, const Eigen::Quaterniond& B)
        :A_(A), B_(B) {}

    template <typename T>
    bool operator()(const T* const _X, const T* const _Y,
                    T* residuals) const
    {
        Eigen::Quaternion<T> A = A_.cast<T>();
        Eigen::Quaternion<T> B = B_.cast<T>();
        Eigen::Quaternion<T> X{_X[3], _X[0], _X[1], _X[2]};
        Eigen::Quaternion<T> Y{_Y[3], _Y[0], _Y[1], _Y[2]};

        Eigen::Quaternion<T> res = B.inverse() * Y.inverse() * A * X;
        residuals[0] = res.x();
        residuals[1] = res.y();
        residuals[2] = res.z();

        return true;
    }

    static ceres::CostFunction* Create(const Eigen::Quaterniond& A, const Eigen::Quaterniond& B)
    {
        return (new ceres::AutoDiffCostFunction<SO3Error, 3, 4, 4>(
                    new SO3Error(A, B)));
    }

    Eigen::Quaterniond A_, B_;
};


int main(int argc, char** argv)
{
    // 要求解的方程是
    // dR_l * R1 * dR_r = R2
    // 移项之后得到
    // R1 * dR_r = dR_l' * R2
    // 记 A=R1, X=dR_r, Y=dR_l', B=R2
    // 求解问题变成 AX=YB

    Eigen::Quaterniond X(0.9, 0.4, 0.3, 0.5);
    X.normalize();

    Eigen::Quaterniond Y(0.1, 0.9, 0.2, 0.2);
    Y.normalize();

    std::vector<Eigen::Quaterniond> A_vec;
    std::vector<Eigen::Quaterniond> B_vec;

    size_t N = 100;
    for(size_t i = 0; i < N; i++) {
        Eigen::Quaterniond A(rand() * 1.0 / RAND_MAX, rand() * 1.0 / RAND_MAX, rand() * 1.0 / RAND_MAX, rand() * 1.0 / RAND_MAX);
        A.normalize();

        Eigen::Quaterniond B = Y.inverse() * A * X;
        B.normalize();

        A_vec.push_back(A);
        B_vec.push_back(B);
    }

    Eigen::Vector4d result_X(rand() * 1.0 / RAND_MAX, rand() * 1.0 / RAND_MAX, rand() * 1.0 / RAND_MAX, rand() * 1.0 / RAND_MAX);
    result_X.normalize();
    Eigen::Vector4d result_Y(rand() * 1.0 / RAND_MAX, rand() * 1.0 / RAND_MAX, rand() * 1.0 / RAND_MAX, rand() * 1.0 / RAND_MAX);
    result_Y.normalize();

    ceres::Problem problem;

    // add parameters
    ceres::LocalParameterization *q_parameterization = new ceres::QuaternionParameterization();
    problem.AddParameterBlock(result_X.data(), 4, q_parameterization);
    problem.AddParameterBlock(result_Y.data(), 4, q_parameterization);

    // add residuals
    for(size_t i = 0; i < A_vec.size(); ++i)
    {
        ceres::CostFunction* cost_function = SO3Error::Create(A_vec[i], B_vec[i]);
        problem.AddResidualBlock(cost_function, nullptr, result_X.data(), result_Y.data());
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << std::endl;

    std::cout << "X        = " << X.coeffs().transpose() << std::endl;
    std::cout << "result_X = " << result_X.transpose() << std::endl;

    std::cout << "Y        = " << Y.coeffs().transpose() << std::endl;
    std::cout << "result_Y = " << result_Y.transpose() << std::endl;

    return 0;
}













