/**
* This file is part of sonar library
* Copyright (C) 2019 Vlasov Aleksey ijonsilent53@gmail.com
* For more information see <https://github.com/DistinctVision/sonar>
**/

#ifndef SONAR_PLANEFINDER_H
#define SONAR_PLANEFINDER_H

#include "types.h"

#include <tuple>
#include <vector>

#include <Eigen/Core>

namespace sonar {

class PlaneFinder
{
public:
    struct Plane
    {
        Eigen::Vector3d normal;
        Eigen::Vector3d point;
        std::vector<int> inliers;
    };

public:
    PlaneFinder();

    double thresholdDistance() const;
    void setThresholdDistance(double thresholdDistance);

    int numberRansacIterations() const;
    void setNumberRansacIterations(int numberRansacIterations);

    Plane find(const points_t & points);

    transformation_t getPlaneTransformation(const Plane & plane,
                                            const point_t & viewPoint) const;

private:
    double m_distanceThreshold;
    int m_numberRansacIterations;

    Plane _computePlane(const points_t & points,
                        const std::vector<int> & indices) const;
};

} // namespace sonar

#endif // SONAR_PLANEFINDER_H
