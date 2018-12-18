#include "test_synthetic_initialization.h"

#include <opengv/types.hpp>

#include <CameraTools/PinholeCamera.h>
#include <Initializator.h>
#include <MapFrame.h>

#include <General/MathUtils.h>

#include <memory>
#include <chrono>
#include <cmath>
#include <random>
#include <Eigen/Eigen>

using namespace std;
using namespace Eigen;
using namespace opengv;
using namespace sonar;

Vector3d generateRandomVector3d(double scale = 1.0)
{
    return Vector3d((rand() % 2000 - 1000),
                    (rand() % 2000 - 1000),
                    (rand() % 2000 - 1000)).normalized() * scale;
}

Matrix3d generateRandomRotationMatrix()
{
    return math_utils::exp_rotationMatrix(generateRandomVector3d(M_PI));
}

bool compare(const Vector3d & a, const Vector3d & b)
{
    Vector3d d = (a - b).array().abs();
    for (int i = 0; i < 3; ++i)
    {
        if (d(i) > 1e-3)
            return false;
    }
    return true;
}

bool compare(const Matrix3d & a, const Matrix3d & b)
{
    Matrix3d d = (a - b).array().abs();
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            if (d(i, j) > 1e-2)
                return false;
        }
    }
    return true;
}

bool test_synthetic_initialization()
{
    shared_ptr<AbstractCamera> camera(new PinholeCamera(Point2d(100.0, 100.0),
                                                        Point2d(50.0, 50.0),
                                                        Point2i(100, 100)));

    Vector3d scenePoint(0.0, 0.0, 100.0);

    Matrix3d real_firstWorldRotation = Matrix3d::Identity();
    Vector3d real_firstWorldPosition = Vector3d::Zero();

    Matrix3d real_secondWorldRotation = generateRandomRotationMatrix();
    //real_secondWorldRotation = math_utils::exp_rotationMatrix(Vector3d(M_PI * 0.5, 0.0, 0.0));
    Vector3d real_secondWorldPosition = scenePoint - real_secondWorldRotation *
            Vector3d(0.0, 0.0, ((rand() % 100) * 1e-1) + 90.0);

    Matrix3d real_thirdWorldRotation = generateRandomRotationMatrix();
    //real_thirdWorldRotation = math_utils::exp_rotationMatrix(Vector3d(-M_PI * 0.5, 0.0, 0.0));
    Vector3d real_thirdWorldPosition = scenePoint - real_thirdWorldRotation *
            Vector3d(0.0, 0.0, ((rand() % 100) * 1e-1) + 90.0);

    Matrix3d first_R = real_firstWorldRotation.inverse();
    Vector3d first_t = - first_R * real_firstWorldPosition;
    Matrix3d second_R = real_secondWorldRotation.inverse();
    Vector3d second_t = - second_R * real_secondWorldPosition;
    Matrix3d third_R = real_thirdWorldRotation.inverse();
    Vector3d third_t = - third_R * real_thirdWorldPosition;

    Vector3d v1 = first_R * scenePoint + first_t;
    Vector3d v2 = second_R * scenePoint + second_t;
    Vector3d v3 = third_R * scenePoint + third_t;

    points_t real_points;
    real_points.reserve(100);
    vector<Point2d> first_image_points(100);
    vector<Point2d> second_image_points(100);
    vector<Point2d> third_image_points(100);
    while (real_points.size() < 100)
    {
        Vector3d point(scenePoint + Vector3d((rand() % 200) * 0.1 - 10.0,
                                             (rand() % 200) * 0.1 - 10.0,
                                             (rand() % 200) * 0.1 - 10.0));

        Vector3d first_local = first_R * point + first_t;
        if (first_local.z() <= 0.0)
            continue;
        Vector3d second_local = second_R * point + second_t;
        if (second_local.z() <= 0.0)
            continue;
        Vector3d third_local = third_R * point + third_t;
        if (third_local.z() <= 0.0)
            continue;
        first_image_points[real_points.size()] = camera->toImagePoint(first_local);
        second_image_points[real_points.size()] = camera->toImagePoint(second_local);
        third_image_points[real_points.size()] = camera->toImagePoint(third_local);
        real_points.push_back(point);
    }

    Initializator::Info info;
    bool successFlag;

    Initializator initializator;
    tie(successFlag, info) = initializator.compute(camera, first_image_points,
                                                   camera, second_image_points,
                                                   camera, third_image_points, false);
    if (!successFlag)
        return false;

   {
        double nScale = third_t.norm() / info.thirdTransfrom.col(3).norm();

        info.secondTransfrom.col(3) *= nScale;
        info.thirdTransfrom.col(3) *= nScale;
        for (Vector3d & point : info.points)
            point *= nScale;
    }

    if (!compare(info.firstTransfrom.block<3, 3>(0, 0), first_R))
        return false;
    if (!compare(info.firstTransfrom.col(3), first_t))
        return false;
    if (!compare(info.secondTransfrom.block<3, 3>(0, 0), second_R))
        return false;
    if (!compare(info.secondTransfrom.col(3), second_t))
        return false;
    if (!compare(info.thirdTransfrom.block<3, 3>(0, 0), third_R))
        return false;
    if (!compare(info.thirdTransfrom.col(3), third_t))
        return false;

    for (size_t i = 0; i < real_points.size(); ++i)
    {
        if (!compare(real_points[i], info.points[i]))
            return false;
    }

    return true;
}
