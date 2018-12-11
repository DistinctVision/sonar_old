#ifndef SONAR_SOURCEFRAME_H
#define SONAR_SOURCEFRAME_H

#include "General/Image.h"

namespace sonar {

class SourceFrame
{
public:
    enum class Type
    {
        Image,
        Texture
    };

    SourceFrame(const ImageRef<uchar> & image);

    Type type() const;

    ConstImage<uchar> image() const;

    SourceFrame copy() const;

private:
    Type m_type;

    ConstImage<uchar> m_image;
};

} // namespace sonar

#endif // SONAR_SOURCEFRAME_H
