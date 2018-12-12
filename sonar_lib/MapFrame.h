#ifndef SONAR_MAPFRAME_H
#define SONAR_MAPFRAME_H

#include <memory>
#include <Eigen/Eigen>

#include "SourceFrame.h"

namespace sonar {

class AbstractCamera;

class SourceFrame;

class MapFrame:
        public SourceFrame
{
public:
    MapFrame(const std::shared_ptr<const SourceFrame> & sourceFrame,
             const std::shared_ptr<const AbstractCamera> & camera,
             const Eigen::Matrix3d & rotation,
             const Eigen::Vector3d & translation);

    std::shared_ptr<const AbstractCamera> camera() const;

    Eigen::Matrix3d rotation() const;
    void setRotation(const Eigen::Matrix3d & rotation);

    Eigen::Vector3d translation() const;
    void setTranslation(const Eigen::Vector3d & translation);

    Eigen::Matrix3d worldRotation() const;
    Eigen::Vector3d worldPosition() const;

private:
    std::shared_ptr<const AbstractCamera> m_camera;
    Eigen::Matrix3d m_rotation;
    Eigen::Vector3d m_translation;
};

} // namespace sonar

#endif // SONAR_MAPFRAME_H
