/**
* This file is part of sonar library
* Copyright (C) 2018 Vlasov Aleksey ijonsilent53@gmail.com
* For more information see <https://github.com/DistinctVision/sonar>
**/

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
        opengv::transformation_t firstTransfrom = opengv::transformation_t::Identity();
        opengv::transformation_t secondTransfrom = opengv::transformation_t::Identity();
        opengv::transformation_t thirdTransfrom = opengv::transformation_t::Identity();

        opengv::points_t points;
    };

    Initializator();

    int minNumberPoints() const;
    void setMinNumberPoints(int minNumberPoints);

    float maxPixelError() const;
    void setMaxPixelError(float maxPixelError);

    int numberRansacIterations() const;
    void setNumberRansacIterations(int numberRansacIterations);

    std::shared_ptr<const PlaneFinder> planeFinder() const;
    std::shared_ptr<PlaneFinder> planeFinder();

    Info lastInitializationInfo() const;

    std::tuple<bool, Info> compute(const std::shared_ptr<const AbstractCamera> & firstCamera,
                                   const std::vector<Point2d> & firstFrame,
                                   const std::shared_ptr<const AbstractCamera> & secondCamera,
                                   const std::vector<Point2d> & secondFrame,
                                   const std::shared_ptr<const AbstractCamera> & thirdCamera,
                                   const std::vector<Point2d> & thirdFrame,
                                   bool alignByPlaneFlag = true);

private:
    int m_minNumberPoints;
    float m_maxPixelError;
    int m_numberRansacIterations;

    Info m_lastInitializationInfo;

    std::shared_ptr<PlaneFinder> m_planeFinder;

    std::tuple<opengv::transformation_t, opengv::transformation_t, std::vector<int>, opengv::points_t>
    _compute(const opengv::bearingVectors_t & firstDirs,
             const opengv::bearingVectors_t & secondDirs,
             const opengv::bearingVectors_t & thirdDirs,
             double firstThreshold, double secondThreshold, double thirdThreshold) const;

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
