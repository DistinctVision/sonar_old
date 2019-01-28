/**
* This file is part of sonar library
* Copyright (C) 2019 Vlasov Aleksey ijonsilent53@gmail.com
* For more information see <https://github.com/DistinctVision/sonar>
**/

#include <sonar/AbstractInitTracker.h>

#include <cassert>

#include <sonar/General/cast.h>

using namespace std;

namespace sonar {

AbstractInitTracker::AbstractInitTracker():
    m_medianDistanceStep(20.0f),
    m_minNumberFeatures(30),
    m_indexStep(0)
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

bool AbstractInitTracker::isFinished() const
{
    return (m_indexStep == 3);
}

int AbstractInitTracker::indexStep() const
{
    return m_indexStep;
}

void AbstractInitTracker::reset()
{
    m_indexStep = 0;
    for (int i = 0; i < 3; ++i)
    {
        m_capturedFrames[i].reset();
        m_capturedFramePoints[i].clear();
    }
}

shared_ptr<const SourceFrame> AbstractInitTracker::capturedFrame(int indexStep) const
{
    assert((indexStep >= 0) && (indexStep <= 2));
    return m_capturedFrames[cast<size_t>(indexStep)];
}

vector<Point2f> AbstractInitTracker::capturedFramePoints(int indexStep) const
{
    assert((indexStep >= 0) && (indexStep <= 2));
    return m_capturedFramePoints[cast<size_t>(indexStep)];
}

void AbstractInitTracker::_capture(const SourceFrame & sourceFrame)
{
    assert(m_indexStep <= 2);
    m_capturedFrames[m_indexStep++] = sourceFrame.sourceCopy();
}

} // namespace sonar
