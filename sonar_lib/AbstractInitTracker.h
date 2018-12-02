#ifndef SONAR_ABSTRACTINITTRACKER_H
#define SONAR_ABSTRACTINITTRACKER_H

#include <tuple>
#include <vector>

#include "General/Point2.h"

#include "SourceFrame.h"

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

    virtual void reset() = 0;

    virtual bool process(const SourceFrame & sourceFrame) = 0;

    virtual bool isFinished() const = 0;

    virtual std::tuple<std::vector<Point2f>,
                       std::vector<Point2f>,
                       std::vector<Point2f>> getMotion() const = 0;

    virtual int indexStep() const = 0;

private:
    float m_medianDistanceStep;
    int m_minNumberFeatures;
};

} // namespace sonar

#endif // SONAR_ABSTRACTINITTRACKER_H
