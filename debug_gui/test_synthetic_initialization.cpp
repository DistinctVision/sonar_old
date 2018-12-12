#include "test_synthetic_initialization.h"

#include <CameraTools/PinholeCamera.h>
#include <Initializator.h>

#include <General/MathUtils.h>

#include <memory>
#include <chrono>
#include <random>
#include <Eigen/Eigen>

using namespace std;
using namespace Eigen;
using namespace sonar;

Vector3d generateRandomVector3d(double scale = 1.0)
{
    return Vector3d((rand() % 2000 - 1000),
                    (rand() % 2000 - 1000),
                    (rand() % 2000 - 1000)).normalized() * scale;
}

Matrix3d generateRandomRotationMatrix()
{
    return math_utils::exp_rotationMatrix(generateRandomVector3d());
}

bool test_synthetic_initialization()
{
    auto camera = make_shared<AbstractCamera>(PinholeCamera(Point2d(100.0, 100.0),
                                                            Point2d(50.0, 50.0),
                                                            Point2i(100, 100)));

    Matrix3d realFirstWorldRotation = Matrix3d::Identity();
    Vector3d realFirstWorldPosition = Vector3d::Zero();

    Matrix3d realSecondWorldRotation = generateRandomRotationMatrix();
    Vector3d realSecondWorldPosition = generateRandomVector3d(10.0);

    Initializator initializator;


    return true;
}
