/**
* This file is part of sonar library
* Copyright (C) 2018 Vlasov Aleksey ijonsilent53@gmail.com
* For more information see <https://github.com/DistinctVision/sonar>
**/

#include "PlaneFinder.h"

#include <cassert>
#include <cmath>
#include <chrono>
#include <random>

#include <Eigen/SVD>

#include <General/cast.h>

using namespace std;
using namespace std::chrono;
using namespace Eigen;
using namespace opengv;

namespace sonar {

PlaneFinder::PlaneFinder():
    m_distanceThreshold(0.01),
    m_numberRansacIterations(50)
{
}

double PlaneFinder::thresholdDistance() const
{
    return m_distanceThreshold;
}

void PlaneFinder::setThresholdDistance(double thresholdDistance)
{
    m_distanceThreshold = thresholdDistance;
}

int PlaneFinder::numberRansacIterations() const
{
    return m_numberRansacIterations;
}

void PlaneFinder::setNumberRansacIterations(int numberRansacIterations)
{
    m_numberRansacIterations = numberRansacIterations;
}

PlaneFinder::Plane PlaneFinder::find(const points_t & points)
{
    assert(points.size() >= 3);

    size_t numberPoints = points.size();

    vector<int> shuffled_indices(numberPoints);
    for (size_t i = 0; i < numberPoints; ++i)
        shuffled_indices[i] = cast<int>(i);
    random_shuffle(shuffled_indices.begin(), shuffled_indices.end());

    mt19937 rnd_gen;
    uniform_int_distribution<int> rnd(0, cast<int>(numberPoints) - 1);

    vector<int> samples(3);

    vector<int> inliers;
    inliers.reserve(numberPoints);

    rnd_gen.seed(duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count());

    double best_error = numeric_limits<double>::max();
    vector<int> best_inliers;

    for (int iteration = 0; iteration < m_numberRansacIterations; ++iteration)
    {
        for (size_t i = 0; i < 3; ++i)
        {
            swap(shuffled_indices[i],
                 shuffled_indices[cast<size_t>(rnd(rnd_gen)) % shuffled_indices.size()]);
            samples[i] = shuffled_indices[i];
        }

        Plane plane = _computePlane(points, samples);
        if (plane.normal.norm() < 1e-4)
            continue;

        inliers.resize(0);

        double error = 0.0;
        for (size_t i = 0; i < numberPoints; ++i)
        {
            double e = fabs(plane.normal.dot(points[i] - plane.point));
            if (e > m_distanceThreshold)
            {
                error += m_distanceThreshold;
                continue;
            }
            error += e;
            inliers.push_back(cast<int>(i));
        }

        if (error < best_error)
        {
            best_error = error;
            best_inliers = move(inliers);
            inliers.reserve(numberPoints);
        }
    }

    if (best_inliers.empty())
    {
        return Plane {
            Vector3d(0.0, 1.0, 0.0),
            Vector3d(0.0, 0.0, 0.0),
            vector<int>()
        };
    }
    Plane best_plane = _computePlane(points, best_inliers);
    best_plane.inliers = move(best_inliers);
    return best_plane;
}

transformation_t PlaneFinder::getPlaneTransformation(const PlaneFinder::Plane & plane,
                                                     const point_t & viewPoint) const
{
    ///TODO process bad situations

    Vector3d delta = viewPoint - plane.point;
    Vector3d planeAxisZ = plane.normal;

    if (planeAxisZ.dot(delta) > 0.0)
    {
        planeAxisZ = - planeAxisZ;
    }

    Vector3d planeAxisX = planeAxisZ.cross(delta).normalized();
    Vector3d planeAxisY = planeAxisX.cross(planeAxisZ);

    transformation_t transformation;
    transformation.col(0) = planeAxisX;
    transformation.col(1) = planeAxisY;
    transformation.col(2) = planeAxisZ;
    transformation.col(3) = plane.point;

    return transformation;
}

PlaneFinder::Plane PlaneFinder::_computePlane(const points_t & points, const vector<int> & indices) const
{
    assert(!indices.empty());

    Plane plane;
    plane.point = Vector3d::Zero();

    for (int index : indices)
        plane.point += points[cast<size_t>(index)];
    plane.point /= cast<double>(indices.size());

    Matrix3d M = Matrix3d::Zero();
    for (int index : indices)
    {
        Vector3d delta = points[cast<size_t>(index)] - plane.point;
        M(0, 0) += delta.x() * delta.x();
        M(0, 1) += delta.x() * delta.y();
        M(0, 2) += delta.x() * delta.z();
        M(1, 0) += delta.y() * delta.x();
        M(1, 1) += delta.y() * delta.y();
        M(1, 2) += delta.y() * delta.z();
        M(2, 0) += delta.z() * delta.x();
        M(2, 1) += delta.z() * delta.y();
        M(2, 2) += delta.z() * delta.z();
    }
    JacobiSVD<Matrix3d> svd(M, ComputeFullV);
    plane.normal = svd.matrixV().col(2).normalized();
    return plane;
}

} // namespace sonar
