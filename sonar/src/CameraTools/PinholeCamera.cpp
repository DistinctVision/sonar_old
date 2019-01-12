/**
* This file is part of sonar library
* Copyright (C) 2019 Vlasov Aleksey ijonsilent53@gmail.com
* For more information see <https://github.com/DistinctVision/sonar>
**/

#include <sonar/CameraTools/PinholeCamera.h>

#include <cassert>
#include <cmath>
#include <limits>
#include <climits>

using namespace std;
using namespace Eigen;

namespace sonar {

PinholeCamera::PinholeCamera(const Point2d & pixelFocalLength,
                             const Point2d & pixelOpticalCenter,
                             const Point2i & imageSize):
    AbstractCamera(imageSize),
    m_pixelFocalLength(pixelFocalLength),
    m_pixelOpticalCenter(pixelOpticalCenter)
{
    assert((fabs(m_pixelFocalLength.x) > numeric_limits<double>::epsilon()) &&
           (fabs(m_pixelFocalLength.y) > numeric_limits<double>::epsilon()));
}

Point2d PinholeCamera::pixelFocalLength() const
{
    return m_pixelFocalLength;
}

Point2d PinholeCamera::pixelOpticalCenter() const
{
    return m_pixelOpticalCenter;
}

Vector3d PinholeCamera::toLocalDir(const Point2d & imagePoint) const
{
    Point2d d = imagePoint - m_pixelOpticalCenter;
    return Eigen::Vector3d(d.x / m_pixelFocalLength.x, d.y / m_pixelFocalLength.y, 1.0);
}

Point2d PinholeCamera::toImagePoint(const Vector3d & localDir) const
{
    if (fabs(localDir.z()) < numeric_limits<double>::epsilon())
        return Point2d(0.0, 0.0);
    Point2d uv(localDir.x() / localDir.z(),
               localDir.y() / localDir.z());
    return Point2d(uv.x * m_pixelFocalLength.x + m_pixelOpticalCenter.x,
                   uv.y * m_pixelFocalLength.y + m_pixelOpticalCenter.y);
}

} // namespace sonar
