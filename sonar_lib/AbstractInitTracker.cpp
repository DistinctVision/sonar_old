#include "AbstractInitTracker.h"

namespace sonar {

AbstractInitTracker::AbstractInitTracker():
    m_medianDistanceStep(20.0f),
    m_minNumberFeatures(30)
{
}

AbstractInitTracker::~AbstractInitTracker() {}

float AbstractInitTracker::medianDistanceStep() const
{
    return m_medianDistanceStep;
}

int AbstractInitTracker::minNumberFeatures() const
{
    return m_minNumberFeatures;
}

void AbstractInitTracker::setMinNumberFeatures(int minNumberFeatures)
{
    m_minNumberFeatures = minNumberFeatures;
}

void AbstractInitTracker::setMedianDistanceStep(float step)
{
    m_medianDistanceStep = step;
}

} // namespace sonar
