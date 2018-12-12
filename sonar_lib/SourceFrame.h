#ifndef SONAR_SOURCEFRAME_H
#define SONAR_SOURCEFRAME_H

#include <memory>

#include "General/Image.h"

namespace sonar {

class SourceFrame
{
public:
    enum class SourceType
    {
        Image,
        Texture
    };

    SourceFrame(const ImageRef<uchar> & image);
    virtual ~SourceFrame();

    SourceType sourceType() const;

    ConstImage<uchar> image() const;

    std::shared_ptr<const SourceFrame> sourceCopy() const;

private:
    SourceType m_sourceType;

    ConstImage<uchar> m_image;
};

} // namespace sonar

#endif // SONAR_SOURCEFRAME_H
