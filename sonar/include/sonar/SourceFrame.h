/**
* This file is part of sonar library
* Copyright (C) 2018 Vlasov Aleksey ijonsilent53@gmail.com
* For more information see <https://github.com/DistinctVision/sonar>
**/

#ifndef SONAR_SOURCEFRAME_H
#define SONAR_SOURCEFRAME_H

#include <memory>

#include <sonar/General/Image.h>

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
