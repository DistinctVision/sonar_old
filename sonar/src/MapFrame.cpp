/**
* This file is part of sonar library
* Copyright (C) 2018 Vlasov Aleksey ijonsilent53@gmail.com
* For more information see <https://github.com/DistinctVision/sonar>
**/

#include <sonar/MapFrame.h>

using namespace std;
using namespace Eigen;

namespace sonar {

MapFrame::MapFrame(const shared_ptr<const SourceFrame> & sourceFrame,
                   const shared_ptr<const AbstractCamera> & camera,
                   const Matrix3d & rotation,
                   const Vector3d & translation):
    SourceFrame(*sourceFrame),
    m_camera(camera),
    m_rotation(rotation),
    m_translation(translation)
{
}

shared_ptr<const AbstractCamera> MapFrame::camera() const
{
    return m_camera;
}

Matrix3d MapFrame::rotation() const
{
    return m_rotation;
}

void MapFrame::setRotation(const Matrix3d & rotation)
{
    m_rotation = rotation;
}

Vector3d MapFrame::translation() const
{
    return m_translation;
}

void MapFrame::setTranslation(const Vector3d & translation)
{
    m_translation = translation;
}

Matrix3d MapFrame::worldRotation() const
{
    return m_rotation.inverse();
}

Vector3d MapFrame::worldPosition() const
{
    return - m_rotation.inverse() * m_translation;
}

} // namespace sonar
