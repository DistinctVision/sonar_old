/**
* This file is part of sonar library
* Copyright (C) 2019 Vlasov Aleksey ijonsilent53@gmail.com
* For more information see <https://github.com/DistinctVision/sonar>
**/

#include "sonar/HomographyInitializator.h"

#include <cassert>
#include <cmath>
#include <chrono>
#include <random>
#include <algorithm>

#include <Eigen/Core>
#include <Eigen/SVD>

#include <sonar/General/cast.h>
#include <sonar/General/MathUtils.h>
#include <sonar/CameraTools/AbstractCamera.h>

using namespace std;
using namespace std::chrono;
using namespace Eigen;
using namespace sonar::math_utils;

namespace sonar {

HomographyInitializator::HomographyInitializator():
    m_numberRansacIterations(400)
{

}

int HomographyInitializator::numberRansacIterations() const
{
    return m_numberRansacIterations;
}

void HomographyInitializator::setNumberRansacIterations(int numberRansacIterations)
{
    m_numberRansacIterations = numberRansacIterations;
}

tuple<bool, AbstractInitializator::Info>
HomographyInitializator::compute(const shared_ptr<const AbstractCamera> & firstCamera, const vector<Point2d> & firstFrame,
                                 const shared_ptr<const AbstractCamera> & secondCamera, const vector<Point2d> & secondFrame,
                                 const shared_ptr<const AbstractCamera> & thirdCamera, const vector<Point2d> & thirdFrame,
                                 bool alignByPlaneFlag)
{
    assert(firstFrame.size() == secondFrame.size());
    assert(secondFrame.size() == thirdFrame.size());

    size_t numberPoints = firstFrame.size();

    bearingVectors_t firstDirs(numberPoints);
    bearingVectors_t secondDirs(numberPoints);
    bearingVectors_t thirdDirs(numberPoints);

    for (size_t i = 0; i < numberPoints; ++i)
    {
        firstDirs[i] = firstCamera->toLocalDir(firstFrame[i]).normalized();
        secondDirs[i] = secondCamera->toLocalDir(secondFrame[i]).normalized();
        thirdDirs[i] = thirdCamera->toLocalDir(thirdFrame[i]).normalized();
    }

    double firstThreshold = (1.0 - cos(atan(cast<double>(maxPixelError()) /
                                       cast<double>(max(firstCamera->imageSize().x,
                                                        firstCamera->imageSize().y)))));
    double secondThreshold = (1.0 - cos(atan(cast<double>(maxPixelError()) /
                                        cast<double>(max(secondCamera->imageSize().x,
                                                         secondCamera->imageSize().y)))));
    double thirdThreshold = (1.0 - cos(atan(cast<double>(maxPixelError()) /
                                       cast<double>(max(thirdCamera->imageSize().x,
                                                        thirdCamera->imageSize().y)))));

    vector<int> inliers;
    points_t points;
    Vector3d planeNormal, planePoint;
    m_lastInitializationInfo.firstTransfrom = transformation_t::Identity();
    tie(m_lastInitializationInfo.secondTransfrom, m_lastInitializationInfo.thirdTransfrom,
        inliers, points,
        planeNormal, planePoint) = _compute(firstDirs, secondDirs, thirdDirs,
                                            firstThreshold, secondThreshold, thirdThreshold);

    if (cast<int>(inliers.size()) >= minNumberPoints())
    {
        transformation_t & firstTransform = m_lastInitializationInfo.firstTransfrom;
        transformation_t & secondTransform = m_lastInitializationInfo.secondTransfrom;
        transformation_t & thirdTransform = m_lastInitializationInfo.thirdTransfrom;
        if (alignByPlaneFlag) // move all coordinates in plane space
        {
            double t;
            if (!_pickPlane(t, planeNormal, planePoint, Vector3d(0.0, 0.0, 1.0), Vector3d(0.0, 0.0, 0.0)))
            {
                return make_tuple(false, m_lastInitializationInfo);
            }
            if (cast<float>(t) < numeric_limits<float>::epsilon())
            {
                return make_tuple(false, m_lastInitializationInfo);
            }

            double scale = 1.0 / t;

            Matrix3d planeRotation;
            {
                ///TODO process bad situations

                Vector3d delta = Vector3d::Zero() - planePoint;
                Vector3d planeAxisZ = planeNormal;

                if (planeAxisZ.dot(delta) < 0.0)
                {
                    planeAxisZ = - planeAxisZ;
                }

                Vector3d planeAxisX = planeAxisZ.cross(delta).normalized();
                Vector3d planeAxisY = planeAxisX.cross(planeAxisZ);

                planeRotation.col(0) = planeAxisX;
                planeRotation.col(1) = planeAxisY;
                planeRotation.col(2) = planeAxisZ;
            }
            Matrix3d invPlaneRotation = planeRotation.inverse();

            planePoint *= scale;
            firstTransform.col(3) *= scale;
            secondTransform.col(3) *= scale;
            thirdTransform.col(3) *= scale;

            for (point_t & point : points)
                point = (invPlaneRotation * (point * scale - planePoint)).eval();

            firstTransform.col(3) = planePoint;
            firstTransform.block<3, 3>(0, 0) = planeRotation;
            secondTransform.col(3) += secondTransform.block<3, 3>(0, 0) * planePoint;
            secondTransform.block<3, 3>(0, 0) *= planeRotation;
            thirdTransform.col(3) += thirdTransform.block<3, 3>(0, 0) * planePoint;
            thirdTransform.block<3, 3>(0, 0) *= planeRotation;
        }

        m_lastInitializationInfo.points = points;

        return make_tuple(true, m_lastInitializationInfo);
    }

    m_lastInitializationInfo = Info();
    return make_tuple(false, m_lastInitializationInfo);
}

tuple<transformation_t, transformation_t,
      vector<int>, points_t,
      Vector3d, Vector3d>
HomographyInitializator::_compute(const bearingVectors_t & firstDirs,
                                  const bearingVectors_t & secondDirs,
                                  const bearingVectors_t & thirdDirs,
                                  double firstThreshold,
                                  double secondThreshold,
                                  double thirdThreshold) const
{
    ///TODO use parallel calculating

    transformation_t firstTransform = transformation_t::Identity();
    transformation_t secondTransform = transformation_t::Identity();
    points_t best_points;
    vector<int> best_inliers;
    Vector3d planeNormal = Vector3d::Zero();
    Vector3d planePoint = Vector3d::Zero();

    size_t numberPoints = firstDirs.size();

    vector<int> shuffled_indices(numberPoints);
    for (size_t i = 0; i < numberPoints; ++i)
        shuffled_indices[i] = cast<int>(i);
    for (auto it = shuffled_indices.begin(); it != shuffled_indices.end(); )
    {
        if ((std::fabs(cast<float>(firstDirs[cast<size_t>(*it)].z())) < numeric_limits<float>::epsilon()) ||
                std::fabs(cast<float>(thirdDirs[cast<size_t>(*it)].z())) < numeric_limits<float>::epsilon())
        {
            it = shuffled_indices.erase(it);
        }
        else
        {
            ++it;
        }
    }
    random_shuffle(shuffled_indices.begin(), shuffled_indices.end());
    numberPoints = shuffled_indices.size();

    mt19937 rnd_gen;
    uniform_int_distribution<int> rnd(0, cast<int>(numberPoints) - 1);

    vector<int> samples(4);

    rnd_gen.seed(cast<size_t>(duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count()));

    points_t points;
    vector<int> inliers;
    points.reserve(numberPoints);
    inliers.reserve(numberPoints);

    double best_score = numeric_limits<double>::max();
    vector<int> best_samples;
    Matrix3d best_homography_b;
    Matrix3d best_homography_c;

    if (cast<int>(numberPoints) < minNumberPoints())
    {
        return make_tuple(firstTransform, secondTransform,
                          vector<int>(), points_t(),
                          planeNormal, planePoint);
    }

    for (int iteration = 0; iteration < m_numberRansacIterations; ++iteration)
    {
        for (size_t i = 0; i < 4; ++i)
        {
            swap(shuffled_indices[i],
                 shuffled_indices[cast<size_t>(rnd(rnd_gen)) % shuffled_indices.size()]);
        }
        copy(shuffled_indices.begin(), shuffled_indices.begin() + 4, samples.begin());

        Matrix3d H_b = _computeHomography(firstDirs, secondDirs, samples);
        Matrix3d H_c = _computeHomography(firstDirs, thirdDirs, samples);
        inliers.resize(0);

        double score = 0.0;
        for (int index : shuffled_indices)
        {
            double e_b = 1.0 - std::fabs(secondDirs[cast<size_t>(index)].dot(
                        (H_b * firstDirs[cast<size_t>(index)]).normalized()));
            if (e_b > secondThreshold)
            {
                score += secondThreshold + thirdThreshold;
                continue;
            }
            double e_c = 1.0 - std::fabs(thirdDirs[cast<size_t>(index)].dot(
                        (H_c * firstDirs[cast<size_t>(index)]).normalized()));
            if (e_c > thirdThreshold)
            {
                score += secondThreshold + thirdThreshold;
                continue;
            }
            score += e_b + e_c;
            inliers.push_back(index);
        }

        if (score < best_score)
        {
            best_score = score;
            best_samples = samples;
            best_inliers = move(inliers);
            best_homography_b = H_b;
            best_homography_c = H_c;
        }
    }

    vector<int> plane_inliers = move(best_inliers);
    vector<_Decomposition> decompositions_b = _decompose(best_homography_b);
    vector<_Decomposition> decompositions_c = _decompose(best_homography_c);

    if (decompositions_b.empty() || decompositions_c.empty())
    {
        return make_tuple(firstTransform, secondTransform,
                          vector<int>(), points_t(),
                          planeNormal, planePoint);
    }

    for (size_t i = 0; i < decompositions_b.size(); )
    {
        const _Decomposition & decompisition = decompositions_b[i];
        points_t points = _computePoints(firstDirs, secondDirs, best_samples,
                                         decompisition.R, decompisition.t);
        int nVisiblePoints = _getNumberVisiblePoints(decompisition,
                                                     firstDirs, secondDirs,
                                                     best_samples, points);
        if (nVisiblePoints < cast<int>(best_samples.size()))
            decompositions_b.erase(decompositions_b.begin() + i);
        else
            ++i;
    }
    for (size_t i = 0; i < decompositions_c.size(); )
    {
        const _Decomposition & decompisition = decompositions_c[i];
        points_t points = _computePoints(firstDirs, thirdDirs, best_samples,
                                         decompisition.R, decompisition.t);
        int nVisiblePoints = _getNumberVisiblePoints(decompisition,
                                                     firstDirs, thirdDirs,
                                                     best_samples, points);
        if (nVisiblePoints < cast<int>(best_samples.size()))
            decompositions_c.erase(decompositions_c.begin() + i);
        else
            ++i;
    }

    _Decomposition best_decomposition_b;
    _Decomposition best_decomposition_c;
    best_score = numeric_limits<double>::max();
    double sumThresholds = firstThreshold + secondThreshold + thirdThreshold;
    for (size_t i = 0; i < decompositions_c.size(); ++i)
    {
        const _Decomposition & decompisition_c = decompositions_c[i];
        points_t points = _computePoints(firstDirs, thirdDirs,
                                         decompisition_c.R, decompisition_c.t);
        for (size_t j = 0; j < decompositions_b.size(); ++j)
        {
            _Decomposition & decompisition_b = decompositions_b[j];
            if (decompisition_b.planeNormal.dot(decompisition_c.planeNormal) < 0.0)
            {
                decompisition_b.planeNormal = - decompisition_b.planeNormal;
                decompisition_b.planeD = - decompisition_b.planeD;
            }
            decompisition_b.t *= decompisition_c.planeD / decompisition_b.planeD;
            decompisition_b.planeD = decompisition_c.planeD;
            inliers.resize(0);
            double score = 0.0;
            for (size_t k = 0; k < points.size(); ++k)
            {
                double f_e = 1.0 - firstDirs[k].dot(points[k].normalized());
                if (f_e > firstThreshold)
                {
                    score += sumThresholds;
                    continue;
                }
                double s_e = 1.0 - secondDirs[k].dot((decompisition_b.R * points[k] +
                                                      decompisition_b.t).normalized());
                if (s_e > secondThreshold)
                {
                    score += sumThresholds;
                    continue;
                }
                double t_e = 1.0 - thirdDirs[k].dot((decompisition_c.R * points[k] +
                                                     decompisition_c.t).normalized());
                if (t_e > secondThreshold)
                {
                    score += sumThresholds;
                    continue;
                }
                score += f_e + s_e + t_e;
                inliers.push_back(cast<int>(k));
            }

            if (score < best_score)
            {
                best_score = score;
                best_decomposition_b = decompisition_b;
                best_decomposition_c = decompisition_c;
                best_inliers = move(inliers);
                best_points = points;
            }
        }
    }

    firstTransform.block<3, 3>(0, 0) = best_decomposition_b.R;
    firstTransform.col(3) = best_decomposition_b.t;
    secondTransform.block<3, 3>(0, 0) = best_decomposition_c.R;
    secondTransform.col(3) = best_decomposition_c.t;

    for (auto it = plane_inliers.begin(); it != plane_inliers.end(); )
    {
        if (std::find(best_inliers.begin(), best_inliers.end(), *it) == best_inliers.end())
            it = plane_inliers.erase(it);
        else
            ++it;
    }

    if (plane_inliers.empty() || best_inliers.empty())
    {
        return make_tuple(firstTransform, secondTransform,
                          vector<int>(), points_t(),
                          planeNormal, planePoint);
    }

    planeNormal = best_decomposition_c.planeNormal;
    planePoint = Vector3d::Zero();
    for (int index: plane_inliers)
    {
        planePoint += best_points[cast<size_t>(index)];
    }
    planePoint /= cast<double>(plane_inliers.size());

    return make_tuple(firstTransform, secondTransform,
                      best_inliers, best_points,
                      planeNormal, planePoint);
}

Matrix3d HomographyInitializator::_computeHomography(const bearingVectors_t & dirs_a,
                                                     const bearingVectors_t & dirs_b,
                                                     const vector<int> & samples) const
{
    MatrixXd M(std::max(cast<int>(samples.size() * 2), 9), 9);
    for (int i = 0; i < cast<int>(samples.size()); ++i)
    {
        const bearingVector_t & dir_a = dirs_a[samples[i]];
        const bearingVector_t & dir_b = dirs_b[samples[i]];

        Vector2d uv_b(dir_b.x() / dir_b.z(), dir_b.y() / dir_b.z());

        M(i * 2, 0) = dir_a.x();
        M(i * 2, 1) = dir_a.y();
        M(i * 2, 2) = dir_a.z();
        M(i * 2, 3) = 0.0;
        M(i * 2, 4) = 0.0;
        M(i * 2, 5) = 0.0;
        M(i * 2, 6) = - dir_a.x() * uv_b.x();
        M(i * 2, 7) = - dir_a.y() * uv_b.x();
        M(i * 2, 8) = - dir_a.z() * uv_b.x();

        M(i * 2 + 1, 0) = 0.0;
        M(i * 2 + 1, 1) = 0.0;
        M(i * 2 + 1, 2) = 0.0;
        M(i * 2 + 1, 3) = dir_a.x();
        M(i * 2 + 1, 4) = dir_a.y();
        M(i * 2 + 1, 5) = dir_a.z();
        M(i * 2 + 1, 6) = - dir_a.x() * uv_b.y();
        M(i * 2 + 1, 7) = - dir_a.y() * uv_b.y();
        M(i * 2 + 1, 8) = - dir_a.z() * uv_b.y();
    }

    if (samples.size() == 4)
    {
        M.row(8).setZero();
    }

    JacobiSVD<MatrixXd> svd(M, ComputeThinV);
    Matrix<double, 9, 1> h = svd.matrixV().col(8);
    return (Matrix3d() << h(0), h(1), h(2), h(3), h(4), h(5), h(6), h(7), h(8)).finished();
}

vector<HomographyInitializator::_Decomposition>
HomographyInitializator::_decompose(const Matrix3d & H) const
{
    JacobiSVD<Matrix3d> svd(H, ComputeFullU | ComputeFullV | ComputeEigenvectors);

    Vector3d W = svd.singularValues();
    double w1 = std::fabs(W(0));
    double w2 = std::fabs(W(1));
    double w3 = std::fabs(W(2));

    Matrix3d U = svd.matrixU();
    Matrix3d V = svd.matrixV();

    double s = U.determinant() * V.determinant();

    double p = w2;

    if ((std::fabs(cast<float>(w1 - w2)) < numeric_limits<float>::epsilon()) ||
            (std::fabs(cast<float>(w2 - w3)) < numeric_limits<float>::epsilon()))
        return vector<HomographyInitializator::_Decomposition>(); // This motion case is not implemented or is degenerate.

    double x1;
    double x2;
    double x3;

    {
        x1 = std::sqrt((w1 * w1 - w2 * w2) / (w1 * w1 - w3 * w3));
        x2    = 0.0;
        x3 = std::sqrt((w2 * w2 - w3 * w3) / (w1 * w1 - w3 * w3));
    }

    const double e1[4] = { 1.0, -1.0,  1.0, -1.0 };
    const double e2[4] = { 1.0,  1.0, -1.0, -1.0 };

    vector<HomographyInitializator::_Decomposition> decompositions(8);
    Vector3d pn, pt;

    // d' > 0:
    double D = s * p;
    for (size_t signs = 0; signs < 4; ++signs)
    {
        _Decomposition & decomposition = decompositions[signs];

        decomposition.planeD = D;
        double sinTheta = (w1 - w3) * x1 * x3 * e1[signs] * e2[signs] / w2;
        double cosTheta = (w1 * x3 * x3 + w3 * x1 * x1) / w2;
        decomposition.R = Matrix3d::Identity();
        decomposition.R(0, 0) = cosTheta;
        decomposition.R(0, 2) = - sinTheta;
        decomposition.R(2, 0) = sinTheta;
        decomposition.R(2, 2) = cosTheta;

        pt(0) = (w1 - w3) * x1 * e1[signs];
        pt(1) = 0.0;
        pt(2) = - (w1 - w3) * x3 * e2[signs];

        pn(0) = x1 * e1[signs];
        pn(1) = x2;
        pn(2) = x3 * e2[signs];
        decomposition.planeNormal = V * pn;

        decomposition.R = (U * decomposition.R * V.transpose() * s).eval();
        decomposition.t = U * pt;
    }
    // d' < 0:
    D = - s * p;
    for (size_t signs = 0; signs < 4; ++signs)
    {
        _Decomposition & decomposition = decompositions[4 + signs];

        decomposition.planeD = D;
        double sinPhi = (w1 + w3) * x1 * x3 * e1[signs] * e2[signs] / w2;
        double cosPhi = (w3 * x1 * x1 - w1 * x3 * x3) / w2;
        decomposition.R = - Matrix3d::Identity();
        decomposition.R(0, 0) = cosPhi;
        decomposition.R(0, 2) = sinPhi;
        decomposition.R(2, 0) = sinPhi;
        decomposition.R(2, 2) = - cosPhi;

        pt(0) = (w1 + w3) * x1 * e1[signs];
        pt(1) = 0.0;
        pt(2) = (w1 + w3) * x3 * e2[signs];

        pn(0) = x1 * e1[signs];
        pn(1) = x2;
        pn(2) = x3 * e2[signs];
        decomposition.planeNormal = V * pn;

        decomposition.R = U * decomposition.R * V.transpose() * s;
        decomposition.t = U * pt;
    }
    return decompositions;
}

int HomographyInitializator::_getNumberVisiblePoints(const HomographyInitializator::_Decomposition & decomposition,
                                                     const bearingVectors_t & dirs_a,
                                                     const bearingVectors_t & dirs_b,
                                                     const vector<int> & indices,
                                                     const points_t & points) const
{
    assert(indices.size() == points.size());
    int numberVisiblePoints = 0;
    for (size_t i = 0; i < indices.size(); ++i)
    {
        const point_t & point = points[i];
        const bearingVector_t & dir_a = dirs_a[cast<size_t>(indices[i])];
        const bearingVector_t & dir_b = dirs_b[cast<size_t>(indices[i])];

        if (cast<float>(dir_a.dot(point)) < numeric_limits<float>::epsilon())
        {
            continue;
        }
        if (cast<float>(dir_b.dot(decomposition.R * point + decomposition.t)) <
                numeric_limits<float>::epsilon())
        {
            continue;
        }
        ++numberVisiblePoints;
    }
    return numberVisiblePoints;
}

points_t HomographyInitializator::_computePoints(const bearingVectors_t & dirs_a,
                                                 const bearingVectors_t & dirs_b,
                                                 const Matrix3d & R, const Vector3d & t) const
{
    points_t points(dirs_a.size());
    Matrix4d M;
    JacobiSVD<Matrix4d> svd;
    for (size_t i = 0; i < points.size(); ++i)
    {
        const bearingVector_t & dir_a = dirs_a[i];
        const bearingVector_t & dir_b = dirs_b[i];

        M(0, 0) = - dir_a.z();
        M(0, 1) = 0.0;
        M(0, 2) = dir_a.x();
        M(0, 3) = 0.0;

        M(1, 0) = 0.0;
        M(1, 1) = - dir_a.z();
        M(1, 2) = dir_a.y();
        M(1, 3) = 0.0;

        M(2, 0) = dir_b.x() * R(2, 0) - dir_b.z() * R(0, 0);
        M(2, 1) = dir_b.x() * R(2, 1) - dir_b.z() * R(0, 1);
        M(2, 2) = dir_b.x() * R(2, 2) - dir_b.z() * R(0, 2);
        M(2, 3) = dir_b.x() * t(2) - dir_b.z() * t(0);

        M(3, 0) = dir_b.y() * R(2, 0) - dir_b.z() * R(1, 0);
        M(3, 1) = dir_b.y() * R(2, 1) - dir_b.z() * R(1, 1);
        M(3, 2) = dir_b.y() * R(2, 2) - dir_b.z() * R(1, 2);
        M(3, 3) = dir_b.y() * t(2) - dir_b.z() * t(1);

        svd.compute(M, ComputeFullV);

        Vector4d p = svd.matrixV().col(3);
        if (std::fabs(cast<float>(p(3))) < numeric_limits<float>::epsilon())
        {
            points[i] = Vector3d::Zero();
        }
        else
        {
            points[i] = p.segment<3>(0) / p(3);
        }
    }
    return points;
}

points_t HomographyInitializator::_computePoints(const bearingVectors_t & dirs_a,
                                                 const bearingVectors_t & dirs_b,
                                                 const vector<int> & indices,
                                                 const Matrix3d & R, const Vector3d & t) const
{
    points_t points(indices.size());
    Matrix4d M;
    JacobiSVD<Matrix4d> svd;
    for (size_t i = 0; i < indices.size(); ++i)
    {
        const bearingVector_t & dir_a = dirs_a[cast<size_t>(indices[i])];
        const bearingVector_t & dir_b = dirs_b[cast<size_t>(indices[i])];

        M(0, 0) = - dir_a.z();
        M(0, 1) = 0.0;
        M(0, 2) = dir_a.x();
        M(0, 3) = 0.0;

        M(1, 0) = 0.0;
        M(1, 1) = - dir_a.z();
        M(1, 2) = dir_a.y();
        M(1, 3) = 0.0;

        M(2, 0) = dir_b.x() * R(2, 0) - dir_b.z() * R(0, 0);
        M(2, 1) = dir_b.x() * R(2, 1) - dir_b.z() * R(0, 1);
        M(2, 2) = dir_b.x() * R(2, 2) - dir_b.z() * R(0, 2);
        M(2, 3) = dir_b.x() * t(2) - dir_b.z() * t(0);

        M(3, 0) = dir_b.y() * R(2, 0) - dir_b.z() * R(1, 0);
        M(3, 1) = dir_b.y() * R(2, 1) - dir_b.z() * R(1, 1);
        M(3, 2) = dir_b.y() * R(2, 2) - dir_b.z() * R(1, 2);
        M(3, 3) = dir_b.y() * t(2) - dir_b.z() * t(1);

        svd.compute(M, ComputeFullV);

        Vector4d p = svd.matrixV().col(3);
        if (std::fabs(cast<float>(p(3))) < numeric_limits<float>::epsilon())
        {
            points[i] = Vector3d::Zero();
        }
        else
        {
            points[i] = p.segment<3>(0) / p(3);
        }
    }
    return points;
}

bool HomographyInitializator::_pickPlane(double & t,
                                         const Vector3d & planeNormal, const Vector3d & planePoint,
                                         const Vector3d & rayDir, const Vector3d & rayPoint) const
{
    t = rayDir.dot(planeNormal);
    if (cast<float>(std::fabs(t)) < std::numeric_limits<float>::epsilon())
        return false;
    t = - ((rayPoint - planePoint).dot(planeNormal)) / t;
    return true;
}

} // namespace sonar
