#include "Initializator.h"

#include <cassert>
#include <cmath>
#include <chrono>
#include <random>

#include <Eigen/SVD>

#include <opengv/relative_pose/CentralRelativeAdapter.hpp>
#include <opengv/relative_pose/methods.hpp>

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

    Matrix3d firstRotation;
    Vector3d firstPosition;
    Matrix3d secondRotation;
    Vector3d secondPosition;
    vector<int> inliers;
    points_t points;
    tie(firstRotation, firstPosition,
        secondRotation, secondPosition,
        inliers, points) = _compute(firstDirs, secondDirs, thirdDirs);

    return !inliers.empty();
}

std::tuple<Matrix3d, Vector3d, Matrix3d, Vector3d, std::vector<int>, points_t>
Initializator::_compute(const bearingVectors_t & firstDirs,
                        const bearingVectors_t & secondDirs,
                        const bearingVectors_t & thirdDirs) const
{
    ///TODO use parallel calculating

    const double threshold = (1.0 - cos(atan(cast<double>(m_maxPixelError) /
                                             cast<double>(min(m_camera->imageSize().x,
                                                              m_camera->imageSize().y)))));

    pair<relative_pose::CentralRelativeAdapter, relative_pose::CentralRelativeAdapter> adapters(
                relative_pose::CentralRelativeAdapter(firstDirs, secondDirs),
                relative_pose::CentralRelativeAdapter(firstDirs, thirdDirs));

    size_t numberPoints = firstDirs.size();

    Eigen::Matrix3d H;
    H << 0.0, -1.0, 0.0,
         1.0,  0.0, 0.0,
         0.0,  0.0, 1.0;

    vector<int> shuffled_indices(cast<size_t>(numberPoints));
    for (size_t i = 0; i < numberPoints; ++i)
        shuffled_indices[i] = cast<int>(i);
    random_shuffle(shuffled_indices.begin(), shuffled_indices.end());

    mt19937 rnd_gen;
    uniform_int_distribution<int> rnd(0, cast<int>(numberPoints) - 1);

    vector<int> samples(5);

    rnd_gen.seed(cast<size_t>(duration_cast<milliseconds>(
                                  system_clock::now().time_since_epoch()).count()));

    JacobiSVD<Matrix3d> SVD;
    vector<pair<Matrix3d, Matrix3d>, Eigen::aligned_allocator<pair<Matrix3d, Matrix3d>>> temp_rotations;
    for (int iteration = 0; iteration < m_numberRansacIterations; ++iteration)
    {
        for (size_t i = 0; i < samples.size(); ++i)
        {
            swap(shuffled_indices[i],
                 shuffled_indices[cast<size_t>(rnd(rnd_gen)) % shuffled_indices.size()]);
            samples[i] = shuffled_indices[i];
        }

        temp_rotations.resize(0);
        essentials_t first_essentials = relative_pose::fivept_nister(adapters.first, samples);
        essentials_t second_essentials = relative_pose::fivept_nister(adapters.second, samples);
        for (auto it_fE = first_essentials.cbegin(); it_fE != first_essentials.cend(); ++it_fE)
        {
            SVD.compute((*it_fE), Eigen::ComputeFullV | Eigen::ComputeFullU);
            Matrix3d fR_a = SVD.matrixU() * H * SVD.matrixV().transpose();
            Matrix3d fR_b = SVD.matrixU() * H.transpose() * SVD.matrixV().transpose();

            if (fR_a.determinant() < 0.0)
                fR_a = - fR_a;
            if (fR_b.determinant() < 0.0)
                fR_b = - fR_b;

            for (auto it_sE = second_essentials.cbegin(); it_sE != second_essentials.cend(); ++it_sE)
            {
                SVD.compute((*it_sE), Eigen::ComputeFullV | Eigen::ComputeFullU);
                Matrix3d sR_a = SVD.matrixU() * H * SVD.matrixV().transpose();
                Matrix3d sR_b = SVD.matrixU() * H.transpose() * SVD.matrixV().transpose();

                if (sR_a.determinant() < 0.0)
                    sR_a = - sR_a;
                if (sR_b.determinant() < 0.0)
                    sR_b = - sR_b;

                temp_rotations.emplace_back(fR_a, sR_a);
                temp_rotations.emplace_back(fR_a, sR_b);
                temp_rotations.emplace_back(fR_b, sR_a);
                temp_rotations.emplace_back(fR_b, sR_b);
            }
        }

        double score = 0.0;
        for (size_t i = 0; i < numberPoints; ++i)
        {

        }
    }
}

} // namespace sonar
