/**
* This file is part of sonar library
* Copyright (C) 2019 Vlasov Aleksey ijonsilent53@gmail.com
* For more information see <https://github.com/DistinctVision/sonar>
**/

#ifndef SONAR_INITIALIZATOR_H
#define SONAR_INITIALIZATOR_H

#if defined(OPENGV_LIB)

#include "AbstractInitializator.h"

namespace sonar {

// forward declaration
class PlaneFinder;

class Initializator:
        public AbstractInitializator
{
public:
    Initializator();

    int numberRansacEssentialsIterations() const;
    void setNumberRansacEssentialsIterations(int numberRansacIterations);

    int numberRansacTransformsIterations() const;
    void setNumberRansacTransformIterations(int numberRansacIterations);

    std::shared_ptr<const PlaneFinder> planeFinder() const;
    std::shared_ptr<PlaneFinder> planeFinder();

    std::tuple<bool, Info> compute(const std::shared_ptr<const AbstractCamera> & firstCamera,
                                   const std::vector<Point2d> & firstFrame,
                                   const std::shared_ptr<const AbstractCamera> & secondCamera,
                                   const std::vector<Point2d> & secondFrame,
                                   const std::shared_ptr<const AbstractCamera> & thirdCamera,
                                   const std::vector<Point2d> & thirdFrame,
                                   bool alignByPlaneFlag = true) override;

private:
    int m_numberRansacEssentialsIterations;
    int m_numberRansacTransformsIterations;

    std::shared_ptr<PlaneFinder> m_planeFinder;

    std::tuple<transformation_t, transformation_t, std::vector<int>, points_t>
    _compute(const bearingVectors_t & firstDirs,
             const bearingVectors_t & secondDirs,
             const bearingVectors_t & thirdDirs,
             double firstThreshold, double secondThreshold, double thirdThreshold) const;

    opengv::transformations_t _convert(const essentials_t & essentials) const;

    void _filterResult(transformations_t & secondTransformations,
                       transformations_t & thirdTransformations,
                       const std::vector<int> & samples,
                       const bearingVectors_t & firstDirs,
                       const bearingVectors_t & secondDirs,
                       const bearingVectors_t & thirdDirs) const;

    bool _pickPlane(double & t,
                    const Eigen::Vector3d & planeNormal, const Eigen::Vector3d & planePoint,
                    const Eigen::Vector3d & rayDir, const Eigen::Vector3d & rayPoint) const;
};

} // namespace sonar

#endif

#endif // SONAR_INITIALIZATOR_H
