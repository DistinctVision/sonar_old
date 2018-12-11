#include "SourceFrame.h"
#include <cassert>

namespace sonar {

SourceFrame::SourceFrame(const ImageRef<uchar> & image):
    m_type(Type::Image),
    m_image(image)
{
}

SourceFrame::Type SourceFrame::type() const
{
    return m_type;
}

ConstImage<uchar> SourceFrame::image() const
{
    return m_image;
}

SourceFrame SourceFrame::copy() const
{
    switch (m_type) {
    case Type::Image:
        return SourceFrame(m_image.copy());
    default:
        break;
    }
    assert(false);
    return SourceFrame(Image<uchar>());
}

} // namespace sonar
