#include "sonar/AbstractInitializator.h"

namespace sonar {

AbstractInitializator::AbstractInitializator():
    m_minNumberPoints(20),
    m_maxPixelError(4.0f)
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

AbstractInitializator::Info AbstractInitializator::lastInitializationInfo() const
{
    return m_lastInitializationInfo;
}

} // namespace sonar
