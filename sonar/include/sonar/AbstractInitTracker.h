/**
* This file is part of sonar library
* Copyright (C) 2019 Vlasov Aleksey ijonsilent53@gmail.com
* For more information see <https://github.com/DistinctVision/sonar>
**/

#ifndef SONAR_ABSTRACTINITTRACKER_H
#define SONAR_ABSTRACTINITTRACKER_H

#include <tuple>
#include <vector>
#include <array>

#include <sonar/General/Point2.h>

#include <sonar/SourceFrame.h>

namespace sonar {

class AbstractInitTracker
{
public:
    AbstractInitTracker();
    virtual ~AbstractInitTracker();

    float medianDistanceStep() const;
    void setMedianDistanceStep(float step);

    int minNumberFeatures() const;
    void setMinNumberFeatures(int minNumberFeatures);

    void reset();

    bool isFinished() const;

    int indexStep() const;

    std::shared_ptr<const SourceFrame> capturedFrame(int indexStep) const;
    std::vector<Point2f> capturedFramePoints(int indexStep) const;

    virtual bool process(const SourceFrame & sourceFrame) = 0;

protected:
    void _capture(const SourceFrame & sourceFrame);

    std::vector<Point2f> m_capturedFramePoints[3];

private:
    float m_medianDistanceStep;
    int m_minNumberFeatures;
    int m_indexStep;

    std::shared_ptr<const SourceFrame> m_capturedFrames[3];
};

} // namespace sonar

#endif // SONAR_ABSTRACTINITTRACKER_H
