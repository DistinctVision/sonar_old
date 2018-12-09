#include "Initializator.h"

#include <cassert>
#include <cmath>
#include <chrono>
#include <random>

#include <Eigen/SVD>

#include <opengv/relative_pose/CentralRelativeAdapter.hpp>
#include <opengv/relative_pose/methods.hpp>
#include <opengv/absolute_pose/CentralAbsoluteAdapter.hpp>
#include <opengv/absolute_pose/methods.hpp>

#include "General/cast.h"
#include "CameraTools/AbstractCamera.h"

using namespace std;
using namespace std::chrono;
using namespace opengv;
using namespace Eigen;

namespace sonar {

Initializator::Initializator():
    m_minNumberPoints(20),
    m_maxPixelError(2.0f),
    m_numberRansacIterations(300)
{
}

shared_ptr<const AbstractCamera> Initializator::camera() const
{
    return m_camera;
}

void Initializator::setCamera(const shared_ptr<const AbstractCamera> & camera)
{
    m_camera = camera;
}

int Initializator::minNumberPoints() const
{
    return m_minNumberPoints;
}

void Initializator::setMinNumberPoints(int minNumberPoints)
{
    m_minNumberPoints = minNumberPoints;
}

float Initializator::maxPixelError() const
{
    return m_maxPixelError;
}

void Initializator::setMaxPixelError(float maxPixelError)
{
    m_maxPixelError = maxPixelError;
}

int Initializator::numberRansacIterations() const
{
    return m_numberRansacIterations;
}

void Initializator::setNumberRansacIterations(int numberRansacIterations)
{
    m_numberRansacIterations = numberRansacIterations;
}

bool Initializator::compute(const vector<Point2d> & firstFrame,
                            const vector<Point2d> & secondFrame,
                            const vector<Point2d> & thirdFrame)
{
    assert(m_camera);
    assert(firstFrame.size() == secondFrame.size());
    assert(secondFrame.size() == thirdFrame.size());

    size_t numberPoints = firstFrame.size();

    bearingVectors_t firstDirs(numberPoints);
    bearingVectors_t secondDirs(numberPoints);
    bearingVectors_t thirdDirs(numberPoints);

    for (size_t i = 0; i < numberPoints; ++i)
    {
        firstDirs[i] = m_camera->toLocalDir(firstFrame[i]).normalized();
        secondDirs[i] = m_camera->toLocalDir(secondFrame[i]).normalized();
        thirdDirs[i] = m_camera->toLocalDir(thirdFrame[i]).normalized();
    }

    Matrix3d secondRotation;
    Vector3d secondPosition;
    Matrix3d thirdRotation;
    Vector3d thirdPosition;
    vector<int> inliers;
    points_t points;
    tie(secondRotation, secondPosition,
        thirdRotation, thirdPosition,
        inliers, points) = _compute(firstDirs, secondDirs, thirdDirs);

    return (cast<int>(inliers.size()) >= m_minNumberPoints);
}

tuple<Matrix3d, Vector3d, Matrix3d, Vector3d, vector<int>, points_t>
Initializator::_compute(const bearingVectors_t & firstDirs,
                        const bearingVectors_t & secondDirs,
                        const bearingVectors_t & thirdDirs) const
{
    ///TODO use parallel calculating

    pair<relative_pose::CentralRelativeAdapter, relative_pose::CentralRelativeAdapter> adapters(
                relative_pose::CentralRelativeAdapter(firstDirs, secondDirs),
                relative_pose::CentralRelativeAdapter(firstDirs, thirdDirs));

    const double threshold = (1.0 - cos(atan(cast<double>(m_maxPixelError) /
                                             cast<double>(max(m_camera->imageSize().x,
                                                              m_camera->imageSize().y)))));

    size_t numberPoints = firstDirs.size();

    vector<int> shuffled_indices(cast<size_t>(numberPoints));
    for (size_t i = 0; i < numberPoints; ++i)
        shuffled_indices[i] = cast<int>(i);
    random_shuffle(shuffled_indices.begin(), shuffled_indices.end());

    mt19937 rnd_gen;
    uniform_int_distribution<int> rnd(0, cast<int>(numberPoints) - 1);

    vector<int> samples(5);

    rnd_gen.seed(cast<size_t>(duration_cast<milliseconds>(
                                  system_clock::now().time_since_epoch()).count()));

    essentials_t essentials;

    points_t points;
    vector<int> inliers;
    points.reserve(numberPoints);
    inliers.reserve(numberPoints);

    double bestError = numeric_limits<double>::max();
    points_t best_points;
    vector<int> best_inliers;
    transformation_t best_secondTransformation;
    transformation_t best_thirdTransformation;

    for (int iteration = 0; iteration < m_numberRansacIterations; ++iteration)
    {
        for (size_t i = 0; i < samples.size(); ++i)
        {
            swap(shuffled_indices[i],
                 shuffled_indices[cast<size_t>(rnd(rnd_gen)) % shuffled_indices.size()]);
            samples[i] = shuffled_indices[i];
        }

        essentials = relative_pose::fivept_nister(adapters.first, samples);
        transformations_t thirdTransforms = _convert(essentials);

        transformations_t secondTransforms;
        _filterResult(secondTransforms, thirdTransforms, samples, firstDirs, secondDirs, thirdDirs);
        assert(secondTransforms.size() == thirdTransforms.size());

        JacobiSVD<Matrix<double, 6, 4>> SVD;
        Matrix<double, 6, 4> M;
        for (size_t i = 0; i < secondTransforms.size(); ++i)
        {
            const opengv::transformation_t & secondTransform = secondTransforms[i];
            const opengv::transformation_t & thirdTransform = thirdTransforms[i];

            points.resize(0);
            inliers.resize(0);
            double error = 0.0;
            for (size_t j = 0; j < numberPoints; ++j)
            {
                const bearingVector_t & firstDir = firstDirs[j];
                const bearingVector_t & secondDir = secondDirs[j];
                const bearingVector_t & thirdDir = thirdDirs[j];

                M.row(0) = firstDir.x() * Vector4d(0.0, 0.0, 1.0, 0.0) -
                                      firstDir.z() * Vector4d(1.0, 0.0, 0.0, 0.0);
                M.row(1) = firstDir.y() * Vector4d(0.0, 0.0, 1.0, 0.0) -
                                      firstDir.z() * Vector4d(0.0, 1.0, 0.0, 0.0);
                M.row(2) = secondDir.x() * secondTransform.row(2) -
                        secondDir.z() * secondTransform.row(0);
                M.row(3) = secondDir.y() * secondTransform.row(2) -
                        secondDir.z() * secondTransform.row(1);
                M.row(4) = thirdDir.x() * thirdTransform.row(2) -
                        thirdDir.z() * thirdTransform.row(0);
                M.row(5) = thirdDir.y() * thirdTransform.row(2) -
                        thirdDir.z() * thirdTransform.row(1);

                SVD.compute(M, ComputeFullV);

                Vector4d X = SVD.matrixV().col(3);
                if (std::fabs(X(3)) < numeric_limits<double>::epsilon())
                {
                    error += threshold * 3.0;
                    continue;
                }
                X /= X(3);

                double firstError = 1.0 - firstDir.dot(X.segment<3>(0).normalized());
                if (firstError > threshold)
                {
                    error += threshold * 3.0;
                    continue;
                }

                Vector3d secondPoint = secondTransform * X;
                double secondError = 1.0 - secondDir.dot(secondPoint.normalized());
                if (secondError > threshold)
                {
                    error += threshold * 3.0;
                    continue;
                }

                Vector3d thirdPoint = thirdTransform * X;
                double thirdError = 1.0 - thirdDir.dot(thirdPoint.normalized());
                if (thirdError > threshold)
                {
                    error += threshold * 3.0;
                    continue;
                }

                error += firstError + secondError + thirdError;
                inliers.push_back(cast<int>(j));
                points.push_back(X.segment<3>(0));
            }

            if (error < bestError)
            {
                bestError = error;
                best_secondTransformation = secondTransform;
                best_thirdTransformation = thirdTransform;
                best_points = move(points);
                best_inliers = move(inliers);
                points.reserve(numberPoints);
                inliers.reserve(numberPoints);
            }
        }
    }

    return make_tuple(best_secondTransformation.block<3, 3>(0, 0), best_secondTransformation.col(3),
                      best_thirdTransformation.block<3, 3>(0, 0), best_thirdTransformation.col(3),
                      move(best_inliers),
                      move(best_points));
}

transformations_t Initializator::_convert(const essentials_t & essentials) const
{
    static const Matrix3d H = (Matrix3d() << 0.0, -1.0, 0.0,
                                             1.0,  0.0, 0.0,
                                             0.0,  0.0, 1.0).finished();

    JacobiSVD<Matrix3d> SVD;
    opengv::transformation_t transformation;
    opengv::transformations_t transformations;
    for (auto it_E = essentials.cbegin(); it_E != essentials.cend(); ++it_E)
    {
        SVD.compute((*it_E), Eigen::ComputeFullV | Eigen::ComputeFullU);
        Matrix3d R_a = SVD.matrixU() * H * SVD.matrixV().transpose();
        Matrix3d R_b = SVD.matrixU() * H.transpose() * SVD.matrixV().transpose();
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
        transformation.col(3) = t_a;
        transformations.push_back(transformation);
        transformation.block<3, 3>(0, 0) = R_a;
        transformation.col(3) = t_b;
        transformations.push_back(transformation);
        transformation.block<3, 3>(0, 0) = R_b;
        transformation.col(3) = t_a;
        transformations.push_back(transformation);
        transformation.block<3, 3>(0, 0) = R_b;
        transformation.col(3) = t_b;
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
        secondDirs[cast<size_t>(samples[4])]
    };
    JacobiSVD<Matrix4d> SVD;
    Matrix4d M;
    secondTransformations.clear();
    secondTransformations.reserve(thirdTransformations.size());
    opengv::points_t samples_points(5);
    for (auto it_T = secondTransformations.cbegin(); it_T != secondTransformations.cend(); )
    {
        bool successFLag = true;

        for (size_t i = 0; i < 5; ++i)
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
            if (cast<float>(samples_points[i].z()) < numeric_limits<float>::epsilon())
            {
                successFLag = false;
                break;
            }
            Vector3d pointInThirdSpace = (*it_T) * X;
            if (cast<float>(pointInThirdSpace.z()) < numeric_limits<float>::epsilon())
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

        absolute_pose::CentralAbsoluteAdapter adapter(adapterSecondDirs, samples_points);
        transformation_t secondTransform = absolute_pose::epnp(adapter);

        for (size_t i = 0; i < 5; ++i)
        {
            Vector3d pointInSecondSpace = secondTransform.block<3, 3>(0, 0) * samples_points[i] +
                    secondTransform.col(3);
            if (cast<float>(pointInSecondSpace.z()) < numeric_limits<float>::epsilon())
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

        thirdTransformations.push_back(secondTransform); // array associated with thirdTransforms
        ++it_T;
    }
}

} // namespace sonar
