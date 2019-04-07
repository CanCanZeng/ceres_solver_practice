#include <ceres/ceres.h>
#include <ceres/rotation.h>

struct ReprojectionErrorWithQuaternions
{
    ReprojectionErrorWithQuaternions(double observedX, double observedY)
        :observedX_(observedX), observedY_(observedY)
    {}

    template <typename T>
    bool operator()(const T* const camera,
                    const T* const point,
                    T* residuals) const
    {
        // camera 的 [0, 1, 2, 3] 是四元数的 [w, x, y, z]
        T p[3];
        ceres::QuaternionRotatePoint(camera, point, p);

        // camera 的 [4, 5, 6] 是 平移， tcw
        p[0] += camera[4];
        p[1] += camera[5];
        p[2] += camera[6];

        const T& fx = camera[7];
        const T& fy = camera[8];
        const T& cx = camera[9];
        const T& cy = camera[10];

        const T predictedX = fx * p[0] / p[2] + cx;
        const T predictedY = fy * p[1] / p[2] + cy;

        residuals[0] = predictedX - observedX_;
        residuals[1] = predictedY - observedY_;

        return true;
    }

    static ceres::CostFunction* Create(const double observedX, const double observedY)
    {
        return (new ceres::AutoDiffCostFunction<ReprojectionErrorWithQuaternions, 2, 11, 3>
        (new ReprojectionErrorWithQuaternions(observedX, observedY)));
    }

    double observedX_;
    double observedY_;
};

struct ReprojectionErrorWithAngleAxis
{
    ReprojectionErrorWithAngleAxis(double observedX, double observedY, double fx, double fy, double cx, double cy)
        :observedX_(observedX), observedY_(observedY), fx_(fx), fy_(fy), cx_(cx), cy_(cy)
    {}

    template <typename T>
    bool operator()(const T* const camera,
                    const T* const point,
                    T* residuals) const
    {
        // camera 的 [0, 1, 2]
        T p[3];
        ceres::AngleAxisRotatePoint(camera, point, p);

        // camera 的 [3, 4, 5] 是 平移， tcw
        p[0] += camera[3];
        p[1] += camera[4];
        p[2] += camera[5];

        const T predictedX = fx_ * p[0] / p[2] + cx_;
        const T predictedY = fy_ * p[1] / p[2] + cy_;

        residuals[0] = predictedX - observedX_;
        residuals[1] = predictedY - observedY_;

        return true;
    }

    static ceres::CostFunction* Create(const double observedX, const double observedY, double fx, double fy, double cx, double cy)
    {
        return (new ceres::AutoDiffCostFunction<ReprojectionErrorWithAngleAxis, 2, 6, 3>
        (new ReprojectionErrorWithAngleAxis(observedX, observedY, fx, fy, cx, cy)));
    }

    double observedX_;
    double observedY_;
    double fx_, fy_, cx_, cy_;
};
