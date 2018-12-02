#ifndef SONAR_ABSTRACTCAMERA_H
#define SONAR_ABSTRACTCAMERA_H

#include <Eigen/Core>

#include "General/Point2.h"

namespace sonar {

class AbstractCamera
{
public:
    AbstractCamera(const Point2i & imageSize);
    virtual ~AbstractCamera();

    Point2i imageSize() const;

    virtual Eigen::Vector3d toLocalDir(const Point2d & imagePoint) const = 0;
    virtual Point2d toImagePoint(const Eigen::Vector3d & localDir) const = 0;

private:
    Point2i m_imageSize;
};

} // namespace sonar

#endif // SONAR_ABSTRACTCAMERA_H
