#ifndef SONAR_INITIALIZATOR_H
#define SONAR_INITIALIZATOR_H

#include <memory>
#include <utility>
#include <vector>
#include <tuple>

#include <Eigen/Core>
#include <opengv/types.hpp>

#include "General/Point2.h"

namespace sonar {

// forward declaration
class AbstractCamera;
class PlaneFinder;

class Initializator
{
public:
    struct Info
    {
        opengv::rotation_t firstRotation;
        opengv::translation_t firstPosition;

        opengv::rotation_t secondRotation;
        opengv::translation_t secondPosition;

        opengv::rotation_t thirdRotation;
        opengv::translation_t thirdPosition;

        opengv::points_t points;
    };

    Initializator();

    std::shared_ptr<const AbstractCamera> camera() const;
    void setCamera(const std::shared_ptr<const AbstractCamera> & camera);

    int minNumberPoints() const;
    void setMinNumberPoints(int minNumberPoints);

    float maxPixelError() const;
    void setMaxPixelError(float maxPixelError);

    int numberRansacIterations() const;
    void setNumberRansacIterations(int numberRansacIterations);

    std::shared_ptr<const PlaneFinder> planeFinder() const;
    std::shared_ptr<PlaneFinder> planeFinder();

    std::tuple<bool, Info> compute(const std::vector<Point2d> & firstFrame,
                                   const std::vector<Point2d> & secondFrame,
                                   const std::vector<Point2d> & thirdFrame);

private:
    std::shared_ptr<const AbstractCamera> m_camera;
    int m_minNumberPoints;
    float m_maxPixelError;
    int m_numberRansacIterations;

    std::shared_ptr<PlaneFinder> m_planeFinder;

    std::tuple<Eigen::Matrix3d, Eigen::Vector3d,
               Eigen::Matrix3d, Eigen::Vector3d,
               std::vector<int>, opengv::points_t>
    _compute(const opengv::bearingVectors_t & firstDirs,
             const opengv::bearingVectors_t & secondDirs,
             const opengv::bearingVectors_t & thirdDirs) const;

    opengv::transformations_t _convert(const opengv::essentials_t & essentials) const;

    void _filterResult(opengv::transformations_t & secondTransformations,
                       opengv::transformations_t & thirdTransformations,
                       const std::vector<int> & samples,
                       const opengv::bearingVectors_t & firstDirs,
                       const opengv::bearingVectors_t & secondDirs,
                       const opengv::bearingVectors_t & thirdDirs) const;
};

} // namespace sonar

#endif // SONAR_INITIALIZATOR_H
