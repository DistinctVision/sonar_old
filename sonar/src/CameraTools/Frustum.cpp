#include "Frustum.h"

#include "General/cast.h"
#include "General/MathUtils.h"

using namespace std;
using namespace Eigen;
using namespace sonar::math_utils;

namespace sonar {

Frustum::Frustum(const shared_ptr<const DepthFrame> & frame):
    Frustum(shared_ptr<const Frame>(frame), frame->depthNear(), frame->depthFar())
{
}

Frustum::Frustum(const shared_ptr<const Frame> & frame, float near, float far)
{
    shared_ptr<const Camera> camera = frame->camera();

    Point2f imageSize = cast<float>(frame->imageSize());

    AlignedBox<float, 2> cam_bb = getBoundedRect<float>({
            camera->unproject(Point2f(0.0f, 0.0f)),
            camera->unproject(Point2f(imageSize.x, 0.0f)),
            camera->unproject(imageSize),
            camera->unproject(Point2f(0.0f,imageSize.y))
        });

    Vector3f u_points[4] = {
        unproject(cam_bb.corner(AlignedBox<float, 2>::TopLeft)),
        unproject(cam_bb.corner(AlignedBox<float, 2>::TopRight)),
        unproject(cam_bb.corner(AlignedBox<float, 2>::BottomRight)),
        unproject(cam_bb.corner(AlignedBox<float, 2>::BottomLeft))
    };

    Vector3f vertices[8];

    for (int i = 0; i < 4; ++i) {
        vertices[i] = frame->toWorld(u_points[i] * near);
        vertices[4 + i] = frame->toWorld(u_points[i] * far);
    }

    // RIGHT PLANE
    if (!_computePlane(m_planes[0], vertices[5], vertices[6], vertices[1])) {
        m_planes[0] = { Vector3f(1.0f, 0.0f, 0.0f), - vertices[1](0) };
    }
    // LEFT PLANE
    if (!_computePlane(m_planes[1], vertices[7], vertices[4], vertices[3])) {
        m_planes[1] = { - m_planes[0].nDir, m_planes[0].nDir.dot(vertices[3]) };
    }
    // BOTTOM PLANE
    if (!_computePlane(m_planes[2], vertices[6], vertices[7], vertices[2])) {
        m_planes[2] = { Vector3f(0.0f, - 1.0f, 0.0f), vertices[2](1) };
    }
    // TOP PLANE
    if (!_computePlane(m_planes[3], vertices[4], vertices[5], vertices[0])) {
        m_planes[3] = { - m_planes[2].nDir, m_planes[2].nDir.dot(vertices[0]) };
    }
    // FRONT PLANE
    if (!_computePlane(m_planes[4], vertices[5], vertices[4], vertices[6])) {
        m_planes[4] = { Vector3f(0.0f, 0.0f, 1.0f), - vertices[4](2) };
    }
    // BACK PLANE
    if (!_computePlane(m_planes[5], vertices[0], vertices[1], vertices[2])) {
        m_planes[5] = { - m_planes[4].nDir, m_planes[4].nDir.dot(vertices[0]) };
    }
}

Frustum::Frustum(const std::shared_ptr<const Camera> & camera,
                 const Matrix3f & rotation, const Vector3f & translation,
                 float near, float far)
{
    Point2f imageSize = camera->imageSize();

    AlignedBox<float, 2> cam_bb = getBoundedRect<float>({
            camera->unproject(Point2f(0.0f, 0.0f)),
            camera->unproject(Point2f(imageSize.x, 0.0f)),
            camera->unproject(imageSize),
            camera->unproject(Point2f(0.0f,imageSize.y))
        });

    Vector3f u_points[4] = {
        unproject(cam_bb.corner(AlignedBox<float, 2>::TopLeft)),
        unproject(cam_bb.corner(AlignedBox<float, 2>::TopRight)),
        unproject(cam_bb.corner(AlignedBox<float, 2>::BottomRight)),
        unproject(cam_bb.corner(AlignedBox<float, 2>::BottomLeft))
    };
    Matrix3f invRotation = rotation.inverse();
    Vector3f invTranslation = - invRotation * translation;

    Vector3f vertices[8];

    for (int i = 0; i < 4; ++i) {
        vertices[i] = invRotation * (u_points[i] * near) + invTranslation;
        vertices[4 + i] = invRotation * (u_points[i] * far) + invTranslation;
    }

    // RIGHT PLANE
    if (!_computePlane(m_planes[0], vertices[5], vertices[6], vertices[1])) {
        m_planes[0] = { Vector3f(1.0f, 0.0f, 0.0f), - vertices[1](0) };
    }
    // LEFT PLANE
    if (!_computePlane(m_planes[1], vertices[7], vertices[4], vertices[3])) {
        m_planes[1] = { - m_planes[0].nDir, m_planes[0].nDir.dot(vertices[3]) };
    }
    // BOTTOM PLANE
    if (!_computePlane(m_planes[2], vertices[6], vertices[7], vertices[2])) {
        m_planes[2] = { Vector3f(0.0f, - 1.0f, 0.0f), vertices[2](1) };
    }
    // TOP PLANE
    if (!_computePlane(m_planes[3], vertices[4], vertices[5], vertices[0])) {
        m_planes[3] = { - m_planes[2].nDir, m_planes[2].nDir.dot(vertices[0]) };
    }
    // FRONT PLANE
    if (!_computePlane(m_planes[4], vertices[5], vertices[4], vertices[6])) {
        m_planes[4] = { Vector3f(0.0f, 0.0f, 1.0f), - vertices[4](2) };
    }
    // BACK PLANE
    if (!_computePlane(m_planes[5], vertices[0], vertices[1], vertices[2])) {
        m_planes[5] = { - m_planes[4].nDir, m_planes[4].nDir.dot(vertices[0]) };
    }
}

bool Frustum::_computePlane(Plane & plane,
                            const Vector3f & v0, const Vector3f & v1, const Vector3f & v2) const
{
    Vector3f nDir = (v1 - v0).cross(v2 - v0);
    float l = nDir.norm();
    if (l < numeric_limits<float>::epsilon())
        return false;
    plane.nDir = nDir / l;
    plane.D = - plane.nDir.dot(v0);
    return true;
}

bool Frustum::pointInFrustum(const Vector3f & point) const
{
    // (A,B,C) - это (X,Y,Z) нормали плоскости. Формула равна нулю, если точка лежит НА
    // плоскости. Если точка ПОЗАДИ плоскости, значение будет отрицательным, если же
    // ПЕРЕД плоскостью - значение будет положительным. Нам нужно проверить, находится
    // ли точка ПЕРЕД плоскостью, так что всё, что нам нужно сделать - пройти через
    // каждую точку и применить к ней формулу плоскости. Результат будет расстоянием
    // до плоскости.

    // Проходим через все стороны пирамиды.
    for (int i = 0; i < 6; ++i) {
        // Применяем формулу плоскости и проверяем, находится ли точка позади плоскости.
        // Если она позади хотя бы одной плоскости из всех, можно возвращать false.
        if ((m_planes[i].nDir.dot(point) + m_planes[i].D) >= 0.0f) {
            // Точка находится позади стороны, так что она НЕ внутри пирамиды
             return false;
        }
    }

    // Иначе точка находится внутри пирамиды, возвращаем true
    return true;
}

bool Frustum::sphereInFrustum(const Vector3f & position, float radius) const
{
    // Эта функция почти идентична предыдущей. Отличие в том, что деперь нам придется
    // учесть ещё и радиус вокруг точки. Например, одно то, что центр сферы находится вне
    // пирамиды, ещё не значит, что и все её точки находятся снаружи. Поэтому вместо проверки,
    // меньше ли результат формулы нуля (<=0), нужно прибавить к нулю отрицательный радиус сферы.

    // Проходим через все стороны пирамиды
    for (int i = 0; i < 6; ++i) {
        // Если центр сферы дальше от плоскости, чем её радиус
        if ((m_planes[i].nDir.dot(position) + m_planes[i].D) >= radius) {
            // То и вся сфера снаружи, возвращаем false
            return false;
        }
    }

    // Иначе сфера внутри
    return true;
}

bool Frustum::boxInFrustum(const Vector3f & box_min, const Vector3f & box_max) const
{
    // Тут работы немного больше, но тоже не слишком много.
    // Нам передаётся центр куба и половина его длинны. Думайте о длинне так же,
    // как о радиусе для сферы. Мы проверяем каждую точку куба на нахождение внутри
    // пирамиды. Если точка находится перед стороной, переходим к следующей сторону.

    for (int i = 0; i < 6; ++i) {
        const Plane & plane = m_planes[i];

        if ((plane.nDir.dot(box_min) + plane.D) < 0.0f)
           continue;
        if ((plane.nDir.dot(box_max) + plane.D) < 0.0f)
           continue;
        if ((plane.nDir.dot(Vector3f(box_max(0), box_min(1), box_min(2))) + plane.D) < 0.0f)
           continue;
        if ((plane.nDir.dot(Vector3f(box_min(0), box_max(1), box_min(2))) + plane.D) < 0.0f)
           continue;
        if ((plane.nDir.dot(Vector3f(box_min(0), box_min(1), box_max(2))) + plane.D) < 0.0f)
           continue;
        if ((plane.nDir.dot(Vector3f(box_max(0), box_max(1), box_min(2))) + plane.D) < 0.0f)
           continue;
        if ((plane.nDir.dot(Vector3f(box_min(0), box_max(1), box_max(2))) + plane.D) < 0.0f)
           continue;
        if ((plane.nDir.dot(Vector3f(box_max(0), box_min(1), box_max(2))) + plane.D) < 0.0f)
           continue;

        // Если мы дошли до сюда, куб не внутри пирамиды.
        return false;
    }

    return true;
}

Frustum::Plane Frustum::getPlane(Frustum::Sides side) const
{
    return m_planes[side];
}


} // namespace sonar
