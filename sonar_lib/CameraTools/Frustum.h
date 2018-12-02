#ifndef SONAR_FRUSTUM_H
#define SONAR_FRUSTUM_H

#include <memory>
#include <utility>

#include <Eigen/Core>
#include "Camera.h"

namespace slam {

class Frustum
{
public:
    // для обозначения сторон:
    enum Sides:int
    {
        RIGHT   = 0,        // Правая сторона пирамиды
        LEFT    = 1,        // Левая сторона пирамиды
        BOTTOM  = 2,        // Нижняя сторона пирамиды
        TOP     = 3,        // Верхняя сторона пирамиды
        FRONT   = 4,        // Передняя сторона пирамиды
        BACK    = 5         // Задняя
    };

    struct Plane
    {
        Eigen::Vector3f nDir;
        float D;
    };

    Frustum(const std::shared_ptr<const Camera> & camera,
            const Eigen::Matrix3f & rotation, const Eigen::Vector3f & translation,
            float near, float far);

    bool pointInFrustum(const Eigen::Vector3f & point) const;
    bool sphereInFrustum(const Eigen::Vector3f & position, float radius) const;
    bool boxInFrustum(const Eigen::Vector3f & box_min, const Eigen::Vector3f & box_max) const;

    Plane getPlane(Sides side) const;

private:
    Plane m_planes[6];

    bool _computePlane(Plane & plane,
                       const Eigen::Vector3f & v0,
                       const Eigen::Vector3f & v1,
                       const Eigen::Vector3f & v2) const;
};

} // namespace sonar

#endif // SONAR_FRUSTUM_H
