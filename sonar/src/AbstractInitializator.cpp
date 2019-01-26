/**
* This file is part of sonar library
* Copyright (C) 2019 Vlasov Aleksey ijonsilent53@gmail.com
* For more information see <https://github.com/DistinctVision/sonar>
**/

#include "sonar/AbstractInitializator.h"

namespace sonar {

AbstractInitializator::AbstractInitializator():
    m_minNumberPoints(8),
    m_maxPixelError(3.0f),
    m_distanceToPlane(3.0)
{
}

AbstractInitializator::~AbstractInitializator()
{
}

int AbstractInitializator::minNumberPoints() const
{
    return m_minNumberPoints;
}

void AbstractInitializator::setMinNumberPoints(int minNumberPoints)
{
    m_minNumberPoints = minNumberPoints;
}

float AbstractInitializator::maxPixelError() const
{
    return m_maxPixelError;
}

void AbstractInitializator::setMaxPixelError(float maxPixelError)
{
    m_maxPixelError = maxPixelError;
}

double AbstractInitializator::distanceToPlane() const
{
    return m_distanceToPlane;
}

void AbstractInitializator::setDistanceToPlane(double distanceToPlane)
{
    m_distanceToPlane = distanceToPlane;
}

AbstractInitializator::Info AbstractInitializator::lastInitializationInfo() const
{
    return m_lastInitializationInfo;
}

} // namespace sonar
