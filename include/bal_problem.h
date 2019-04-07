#include <string>
#include <ceres/ceres.h>

class BALProblem
{
public:
    ~BALProblem()
    {
        delete[] pointIndex_;
        delete[] cameraIndex_;
        delete[] observations_;
        delete[] parameters_;
    }

    uint32_t GetNumberOfObservations() const {return numOfObservations_;}
    const double* Observations()        const {return observations_;}
    double* MutableCameras()                    {return parameters_;}
    double* MutablePoints()                     {return parameters_ + 11 * numOfCameras_;}

    double* MutableCameraForObservation(uint32_t i)
    {
        return MutableCameras() + cameraIndex_[i] * 11;
    }

    double* MutablePointForObservation(uint32_t i)
    {
        return MutablePoints() + pointIndex_[i] * 3;
    }

public:
    uint32_t numOfCameras_;
    uint32_t numOfPoints_;
    uint32_t numOfObservations_;
    uint32_t numOfParameters_;

    uint32_t* pointIndex_;
    uint32_t* cameraIndex_;
    double* observations_;
    double* parameters_;
};
