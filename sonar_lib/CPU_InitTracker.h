#ifndef SONAR_CPU_INITTRACKER_H
#define SONAR_CPU_INITTRACKER_H

#include "AbstractInitTracker.h"

#include "ImageTools/FeatureDetector.h"
#include "ImageTools/OpticalFlow.h"

namespace sonar {

class CPU_InitTracker:
        public AbstractInitTracker
{
public:
    CPU_InitTracker();

    const FeatureDetector & featureTracker() const;
    FeatureDetector & featureTracker();

    const OpticalFlow & opticalFlow() const;
    OpticalFlow & opticalFlow();

    bool process(const SourceFrame & sourceFrame) override;

private:
    FeatureDetector m_featureDetector;
    OpticalFlow m_opticalFlow;

    std::vector<Point2f> m_currentPoints;

    float _medianDistance(const std::vector<Point2f> & pointsA,
                          const std::vector<Point2f> & pointsB) const;
};

} // namespace sonar

#endif // SONAR_CPU_INITTRACKER_H
