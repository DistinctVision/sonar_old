#include "SourceFrame.h"

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

} // namespace sonar
