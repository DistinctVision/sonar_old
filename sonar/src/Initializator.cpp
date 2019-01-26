/**
* This file is part of sonar library
* Copyright (C) 2019 Vlasov Aleksey ijonsilent53@gmail.com
* For more information see <https://github.com/DistinctVision/sonar>
**/
#include <sonar/Initializator.h>

#if defined(OPENGV_LIB)

#include <cassert>
#include <cmath>
#include <chrono>
#include <random>

#include <Eigen/SVD>

#include <opengv/relative_pose/CentralRelativeAdapter.hpp>
#include <opengv/relative_pose/methods.hpp>
#include <opengv/absolute_pose/CentralAbsoluteAdapter.hpp>
#include <opengv/absolute_pose/methods.hpp>

#include <sonar/General/cast.h>
#include <sonar/CameraTools/AbstractCamera.h>

#include <sonar/PlaneFinder.h>

using namespace std;
using namespace std::chrono;
using namespace Eigen;

namespace sonar {

Initializator::Initializator():
    m_numberRansacEssentialsIterations(300),
    m_numberRansacTransformsIterations(30)
{
    m_planeFinder = make_shared<PlaneFinder>();
}

int Initializator::numberRansacEssentialsIterations() const
{
    return m_numberRansacEssentialsIterations;
}

void Initializator::setNumberRansacEssentialsIterations(int numberRansacIterations)
{
    m_numberRansacEssentialsIterations = numberRansacIterations;
}

int Initializator::numberRansacTransformsIterations() const
{
    return m_numberRansacTransformsIterations;
}

void Initializator::setNumberRansacTransformIterations(int numberRansacIterations)
{
    m_numberRansacTransformsIterations = numberRansacIterations;
}

shared_ptr<const PlaneFinder> Initializator::planeFinder() const
{
    return m_planeFinder;
}

shared_ptr<PlaneFinder> Initializator::planeFinder()
{
    return m_planeFinder;
}

tuple<bool, Initializator::Info> Initializator::compute(const std::shared_ptr<const AbstractCamera> & firstCamera,
                                                        const vector<Point2d> & firstFrame,
                                                        const std::shared_ptr<const AbstractCamera> & secondCamera,
                                                        const vector<Point2d> & secondFrame,
                                                        const std::shared_ptr<const AbstractCamera> & thirdCamera,
                                                        const vector<Point2d> & thirdFrame,
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
    m_lastInitializationInfo.firstTransfrom = transformation_t::Identity();
    tie(m_lastInitializationInfo.secondTransfrom, m_lastInitializationInfo.thirdTransfrom,
        inliers, points) = _compute(firstDirs, secondDirs, thirdDirs,
                                    firstThreshold, secondThreshold, thirdThreshold);

    if (cast<int>(inliers.size()) >= minNumberPoints())
    {
        PlaneFinder::Plane plane = m_planeFinder->find(points);

        transformation_t & firstTransform = m_lastInitializationInfo.firstTransfrom;
        transformation_t & secondTransform = m_lastInitializationInfo.secondTransfrom;
        transformation_t & thirdTransform = m_lastInitializationInfo.thirdTransfrom;
        if (!plane.inliers.empty() && alignByPlaneFlag) // move all coordinates in plane space
        {
            transformation_t planeTransformation = m_planeFinder->getPlaneTransformation(plane, Vector3d::Zero());
            Vector3d planePoint = planeTransformation.col(3);

            double scale = distanceToPlane() / planePoint.norm();
            double invScale = 1.0 / scale;

            Matrix3d planeRotation = planeTransformation.block<3, 3>(0, 0);
            Matrix3d invPlaneRotation = planeRotation.inverse();

            planePoint *= scale;
            firstTransform.col(3) *= invScale;
            secondTransform.col(3) *= invScale;
            thirdTransform.col(3) *= invScale;

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

tuple<transformation_t, transformation_t, vector<int>, points_t>
Initializator::_compute(const bearingVectors_t & firstDirs,
                        const bearingVectors_t & secondDirs,
                        const bearingVectors_t & thirdDirs,
                        double firstThreshold, double secondThreshold, double thirdThreshold) const
{
    ///TODO use parallel calculating

    size_t numberPoints = firstDirs.size();

    vector<int> shuffled_indices(numberPoints);
    for (size_t i = 0; i < numberPoints; ++i)
        shuffled_indices[i] = cast<int>(i);
    random_shuffle(shuffled_indices.begin(), shuffled_indices.end());

    mt19937 rnd_gen;
    uniform_int_distribution<int> rnd(0, cast<int>(numberPoints) - 1);

    vector<int> samples(5);

    rnd_gen.seed(cast<size_t>(duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count()));

    essentials_t essentials;

    points_t points;
    vector<int> inliers;
    points.reserve(numberPoints);
    inliers.reserve(numberPoints);

    double best_score = numeric_limits<double>::max();
    points_t best_points;
    vector<int> best_inliers;
    essential_t best_essentialMatrix;

    opengv::relative_pose::CentralRelativeAdapter adapter(firstDirs, thirdDirs);

    for (int iteration = 0; iteration < m_numberRansacEssentialsIterations; ++iteration)
    {
        for (size_t i = 0; i < 5; ++i)
        {
            swap(shuffled_indices[i],
                 shuffled_indices[cast<size_t>(rnd(rnd_gen)) % shuffled_indices.size()]);
        }
        copy(shuffled_indices.begin(), shuffled_indices.begin() + 5, samples.begin());

        essentials = opengv::relative_pose::fivept_nister(adapter, samples);

        for (auto itE = essentials.cbegin(); itE != essentials.cend(); ++itE)
        {
            double score = 0.0;
            for (size_t i = 0; i < numberPoints; ++i)
            {
                const bearingVector_t & firstDir = firstDirs[i];
                const bearingVector_t & thirdDir = thirdDirs[i];

                double e = std::fabs(thirdDir.transpose() * itE->transpose() * firstDir);

                score += std::min(e, firstThreshold) + std::min(e, thirdThreshold);
            }

            if (score < best_score)
            {
                best_score = score;
                best_essentialMatrix = (*itE);
                inliers = samples;
            }
        }

        iteration += std::max(cast<int>(essentials.size()), 1);
    }

    transformation_t best_secondTransformation;
    transformation_t best_thirdTransformation;
    transformations_t possibles_thirdTransforms = _convert({ best_essentialMatrix });
    samples = inliers;
    for (size_t i = 0; i < numberPoints; ++i)
        shuffled_indices[i] = cast<int>(i);
    for (size_t i = 0; i < samples.size(); ++i)
    {
        shuffled_indices[cast<size_t>(samples[i])] = shuffled_indices.back();
        shuffled_indices.resize(shuffled_indices.size() - 1);
    }
    samples.resize(6);
    best_score = numeric_limits<double>::max();
    JacobiSVD<Matrix<double, 4, 4>> SVD;
    Matrix<double, 4, 4> M;
    double sumThresholds = firstThreshold + secondThreshold + thirdThreshold;
    for (int iteration = 0; iteration < m_numberRansacTransformsIterations; )
    {
        swap(shuffled_indices[0],
             shuffled_indices[cast<size_t>(rnd(rnd_gen)) % shuffled_indices.size()]);
        samples[5] = shuffled_indices[0];

        transformations_t thirdTransforms = possibles_thirdTransforms;
        transformations_t secondTransforms;
        _filterResult(secondTransforms, thirdTransforms, samples, firstDirs, secondDirs, thirdDirs);
        assert(secondTransforms.size() == thirdTransforms.size());

        for (size_t i = 0; i < secondTransforms.size(); ++i)
        {
            const transformation_t & secondTransform = secondTransforms[i];
            const transformation_t & thirdTransform = thirdTransforms[i];

            points.resize(0);
            inliers.resize(0);
            double score = 0.0;
            for (size_t j = 0; j < numberPoints; ++j)
            {
                const bearingVector_t & firstDir = firstDirs[j];
                const bearingVector_t & secondDir = secondDirs[j];
                const bearingVector_t & thirdDir = thirdDirs[j];

                M.row(0) = firstDir.x() * Vector4d(0.0, 0.0, 1.0, 0.0) -
                        firstDir.z() * Vector4d(1.0, 0.0, 0.0, 0.0);
                M.row(1) = firstDir.y() * Vector4d(0.0, 0.0, 1.0, 0.0) -
                        firstDir.z() * Vector4d(0.0, 1.0, 0.0, 0.0);
                M.row(2) = thirdDir.x() * thirdTransform.row(2) -
                        thirdDir.z() * thirdTransform.row(0);
                M.row(3) = thirdDir.y() * thirdTransform.row(2) -
                        thirdDir.z() * thirdTransform.row(1);

                SVD.compute(M, ComputeFullV);

                Vector4d X = SVD.matrixV().col(3);
                if (std::fabs(X(3)) < numeric_limits<double>::epsilon())
                {
                    score += sumThresholds;
                    continue;
                }
                X /= X(3);

                double firstScore = 1.0 - firstDir.dot(X.segment<3>(0).normalized());
                if (firstScore > firstThreshold)
                {
                    score += sumThresholds;
                    continue;
                }

                Vector3d secondPoint = secondTransform * X;
                double secondScore = 1.0 - secondDir.dot(secondPoint.normalized());
                if (secondScore > secondThreshold)
                {
                    score += sumThresholds;
                    continue;
                }

                Vector3d thirdPoint = thirdTransform * X;
                double thirdScore = 1.0 - thirdDir.dot(thirdPoint.normalized());
                if (thirdScore > thirdThreshold)
                {
                    score += sumThresholds;
                    continue;
                }

                score += firstScore + secondScore + thirdScore;
                inliers.push_back(cast<int>(j));
                points.push_back(X.segment<3>(0));
            }

            if (score < best_score)
            {
                best_score = score;
                best_secondTransformation = secondTransform;
                best_thirdTransformation = thirdTransform;
                best_points = move(points);
                best_inliers = move(inliers);
                points.reserve(numberPoints);
                inliers.reserve(numberPoints);
            }
        }

        iteration += std::max(cast<int>(secondTransforms.size()), 1);
    }

    return make_tuple(best_secondTransformation, best_thirdTransformation,
                      move(best_inliers), move(best_points));
}

transformations_t Initializator::_convert(const essentials_t & essentials) const
{
    static const Matrix3d H = (Matrix3d() << 0.0, -1.0, 0.0,
                                             1.0,  0.0, 0.0,
                                             0.0,  0.0, 1.0).finished();

    JacobiSVD<Matrix3d> SVD;
    transformation_t transformation;
    transformations_t transformations;
    for (auto it_E = essentials.cbegin(); it_E != essentials.cend(); ++it_E)
    {
        SVD.compute((*it_E), Eigen::ComputeFullV | Eigen::ComputeFullU);
        Matrix3d R_a = SVD.matrixV() * H.transpose() * SVD.matrixU().transpose();
        Matrix3d R_b = SVD.matrixV() * H * SVD.matrixU().transpose();
        Eigen::Vector3d singularValues = SVD.singularValues();

        // check for bad essential matrix
        if (singularValues(2) > 1e-3)
            continue; // singularity constraints not applied -> removed because too harsh
        if (singularValues(1) < 0.75 * singularValues(0))
            continue; // bad essential matrix -> removed because too harsh

        // maintain scale
        double scale = singularValues(0);

        translation_t t_a = scale * SVD.matrixU().col(2);
        translation_t t_b = - t_a;

        if (R_a.determinant() < 0.0)
            R_a = - R_a;
        if (R_b.determinant() < 0.0)
            R_b = - R_b;

        transformation.block<3, 3>(0, 0) = R_a;
        transformation.col(3) = - R_a * t_a;
        transformations.push_back(transformation);
        transformation.block<3, 3>(0, 0) = R_a;
        transformation.col(3) = - R_a * t_b;
        transformations.push_back(transformation);
        transformation.block<3, 3>(0, 0) = R_b;
        transformation.col(3) = - R_b * t_a;
        transformations.push_back(transformation);
        transformation.block<3, 3>(0, 0) = R_b;
        transformation.col(3) = - R_b * t_b;
        transformations.push_back(transformation);
    }
    return transformations;
}

void Initializator::_filterResult(transformations_t & secondTransformations,
                                  transformations_t & thirdTransformations,
                                  const vector<int> & samples,
                                  const bearingVectors_t & firstDirs,
                                  const bearingVectors_t & secondDirs,
                                  const bearingVectors_t & thirdDirs) const
{
    bearingVectors_t adapterSecondDirs = {
        secondDirs[cast<size_t>(samples[0])],
        secondDirs[cast<size_t>(samples[1])],
        secondDirs[cast<size_t>(samples[2])],
        secondDirs[cast<size_t>(samples[3])],
        secondDirs[cast<size_t>(samples[4])],
        secondDirs[cast<size_t>(samples[5])],
    };
    JacobiSVD<Matrix4d> SVD;
    Matrix4d M;
    secondTransformations.clear();
    secondTransformations.reserve(thirdTransformations.size());
    points_t samples_points(6);
    for (auto it_T = thirdTransformations.cbegin(); it_T != thirdTransformations.cend(); )
    {
        bool successFLag = true;

        for (size_t i = 0; i < 6; ++i)
        {
            const Vector3d & firstDir = firstDirs[cast<size_t>(samples[i])];
            const Vector3d & thirdDir = thirdDirs[cast<size_t>(samples[i])];

            M.row(0) = firstDir.x() * Vector4d(0.0, 0.0, 1.0, 0.0) -
                                  firstDir.z() * Vector4d(1.0, 0.0, 0.0, 0.0);
            M.row(1) = firstDir.y() * Vector4d(0.0, 0.0, 1.0, 0.0) -
                                  firstDir.z() * Vector4d(0.0, 1.0, 0.0, 0.0);
            M.row(2) = thirdDir.x() * it_T->row(2) - thirdDir.z() * it_T->row(0);
            M.row(3) = thirdDir.y() * it_T->row(2) - thirdDir.z() * it_T->row(1);

            SVD.compute(M, ComputeFullV);

            Vector4d X = SVD.matrixV().col(3);
            if (std::fabs(X(3)) < numeric_limits<double>::epsilon())
            {
                successFLag = false;
                break;
            }
            X /= X(3);
            samples_points[i] = X.segment<3>(0);
            if (cast<float>(firstDir.dot(samples_points[i])) < numeric_limits<float>::epsilon())
            {
                successFLag = false;
                break;
            }
            Vector3d pointInThirdSpace = it_T->block<3, 3>(0, 0).transpose() * (samples_points[i] - it_T->col(3));
            if (cast<float>(thirdDir.dot(pointInThirdSpace)) < numeric_limits<float>::epsilon())
            {
                successFLag = false;
                break;
            }
        }
        if (!successFLag) // skip failed transformations
        {
            it_T = thirdTransformations.erase(it_T);
            continue;
        }

        opengv::absolute_pose::CentralAbsoluteAdapter adapter(adapterSecondDirs, samples_points);
        transformation_t inv_secondTransform = opengv::absolute_pose::epnp(adapter);
        transformation_t secondTransform;
        secondTransform.block<3, 3>(0, 0) = inv_secondTransform.block<3, 3>(0, 0).transpose();
        secondTransform.col(3) = - secondTransform.block<3, 3>(0, 0) * inv_secondTransform.col(3);

        for (size_t i = 0; i < 6; ++i)
        {
            const Vector3d & secondDir = secondDirs[cast<size_t>(samples[i])];
            Vector3d pointInSecondSpace =
                    secondTransform.block<3, 3>(0, 0) * samples_points[i] + secondTransform.col(3);
            if (cast<float>(secondDir.dot(pointInSecondSpace)) < numeric_limits<float>::epsilon())
            {
                successFLag = false;
                break;
            }
        }
        if (!successFLag)
        {
            it_T = thirdTransformations.erase(it_T);
            continue;
        }

        secondTransformations.push_back(secondTransform); // array associated with thirdTransforms
        ++it_T;
    }
}

bool Initializator::_pickPlane(double & t,
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

#endif
