#include "AbstractCamera.h"

#include <cassert>

namespace sonar {

AbstractCamera::AbstractCamera(const Point2i & imageSize):
    m_imageSize(imageSize)
{
    assert((m_imageSize.x > 0) && (m_imageSize.y > 0));
}

AbstractCamera::~AbstractCamera() {}

Point2i AbstractCamera::imageSize() const
{
    return m_imageSize;
}

} // namespace sonar
