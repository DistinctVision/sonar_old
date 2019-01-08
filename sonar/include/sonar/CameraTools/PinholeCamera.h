/**
* This file is part of sonar library
* Copyright (C) 2018 Vlasov Aleksey ijonsilent53@gmail.com
* For more information see <https://github.com/DistinctVision/sonar>
**/

#ifndef SONAR_PINHOLECAMERA_H
#define SONAR_PINHOLECAMERA_H

#include "AbstractCamera.h"

namespace sonar {

class PinholeCamera:
        public AbstractCamera
{
public:
    PinholeCamera(const Point2d & pixelFocalLength,
                  const Point2d & pixelOpticalCenter,
                  const Point2i & imageSize);

    Point2d pixelFocalLength() const;
    Point2d pixelOpticalCenter() const;

    Eigen::Vector3d toLocalDir(const Point2d & imagePoint) const override;
    Point2d toImagePoint(const Eigen::Vector3d & localDir) const override;

private:
    Point2i m_imageSize;
    Point2d m_pixelFocalLength;
    Point2d m_pixelOpticalCenter;
};

} // namespace sonar

#endif // SONAR_PINHOLECAMERA_H
