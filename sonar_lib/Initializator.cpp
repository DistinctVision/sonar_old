#include "Initializator.h"
#include <cassert>

#include <Eigen/SVD>

#include <opengv/types.hpp>

#include "General/cast.h"
#include "CameraTools/AbstractCamera.h"

using namespace std;
using namespace opengv;

namespace sonar {

Initializator::Initializator()
{
}

shared_ptr<const AbstractCamera> Initializator::camera() const
{
    return m_camera;
}

void Initializator::setCamera(const shared_ptr<const AbstractCamera> & camera)
{
    m_camera = camera;
}

bool Initializator::init(const vector<Point2d> & firstFrame,
                         const vector<Point2d> & secondFrame,
                         const vector<Point2d> & thirdFrame)
{
    assert(m_camera);
    assert(firstFrame.size() == secondFrame.size());
    assert(secondFrame.size() == thirdFrame.size());

    size_t numberPoints = firstFrame.size();

    bearingVector_t firstDirs(numberPoints);
    bearingVector_t secondDirs(numberPoints);
    bearingVector_t thirdDirs(numberPoints);

    for (size_t i = 0; i < numberPoints; ++i)
    {
        firstDirs[i] = m_camera->toLocalDir(firstFrame[i]);
        secondDirs[i] = m_camera->toLocalDir(secondFrame[i]);
        thirdDirs[i] = m_camera->toLocalDir(thirdFrame[i]);
    }
}

} // namespace sonar
