/**
* This file is part of sonar library
* Copyright (C) 2019 Vlasov Aleksey ijonsilent53@gmail.com
* For more information see <https://github.com/DistinctVision/sonar>
**/

#ifndef SONAR_ABSTRACTINITIALIZATOR_H
#define SONAR_ABSTRACTINITIALIZATOR_H

#include "types.h"

#include <memory>
#include <utility>
#include <vector>
#include <tuple>

#include <Eigen/Core>

#include <sonar/General/Point2.h>

namespace sonar {

// forward declaration
class AbstractCamera;

class AbstractInitializator
{
public:
    struct Info
    {
        transformation_t firstTransfrom = transformation_t::Identity();
        transformation_t secondTransfrom = transformation_t::Identity();
        transformation_t thirdTransfrom = transformation_t::Identity();

        points_t points;
    };

    AbstractInitializator();
    virtual ~AbstractInitializator();

    int minNumberPoints() const;
    void setMinNumberPoints(int minNumberPoints);

    float maxPixelError() const;
    void setMaxPixelError(float maxPixelError);

    Info lastInitializationInfo() const;

    virtual std::tuple<bool, Info> compute(const std::shared_ptr<const AbstractCamera> & firstCamera,
                                           const std::vector<Point2d> & firstFrame,
                                           const std::shared_ptr<const AbstractCamera> & secondCamera,
                                           const std::vector<Point2d> & secondFrame,
                                           const std::shared_ptr<const AbstractCamera> & thirdCamera,
                                           const std::vector<Point2d> & thirdFrame,
                                           bool alignByPlaneFlag = true) = 0;

protected:
    Info m_lastInitializationInfo;

private:
    int m_minNumberPoints;
    float m_maxPixelError;
};

} // namespace sonar

#endif // SONAR_ABSTRACTINITIALIZATOR_H
