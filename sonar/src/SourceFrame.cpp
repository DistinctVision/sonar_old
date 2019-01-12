/**
* This file is part of sonar library
* Copyright (C) 2019 Vlasov Aleksey ijonsilent53@gmail.com
* For more information see <https://github.com/DistinctVision/sonar>
**/

#include <sonar/SourceFrame.h>
#include <cassert>

using namespace std;

namespace sonar {

SourceFrame::SourceFrame(const ImageRef<uchar> & image):
    m_sourceType(SourceType::Image),
    m_image(image)
{
}

SourceFrame::~SourceFrame()
{
}

SourceFrame::SourceType SourceFrame::sourceType() const
{
    return m_sourceType;
}

ConstImage<uchar> SourceFrame::image() const
{
    return m_image;
}

shared_ptr<const SourceFrame> SourceFrame::sourceCopy() const
{
    switch (m_sourceType) {
    case SourceType::Image:
        return make_shared<SourceFrame>(m_image.copy());
    default:
        break;
    }
    assert(false);
    return make_shared<SourceFrame>(Image<uchar>());
}

} // namespace sonar
