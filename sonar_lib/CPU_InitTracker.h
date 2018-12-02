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

    void reset() override;

    bool process(const SourceFrame & sourceFrame) override;

    bool isFinished() const override;

    std::tuple<std::vector<Point2f>,
               std::vector<Point2f>,
               std::vector<Point2f>> getMotion() const override;

    int indexStep() const override;

    const FeatureDetector & featureTracker() const;
    FeatureDetector & featureTracker();

    const OpticalFlow & opticalFlow() const;
    OpticalFlow & opticalFlow();

private:
    FeatureDetector m_featureDetector;
    OpticalFlow m_opticalFlow;

    std::vector<Point2f> m_capturedFramePoints[3];
    int m_indexStep;

    float _medianDistance(const std::vector<Point2f> & pointsA,
                          const std::vector<Point2f> & pointsB) const;
};

} // namespace sonar

#endif // SONAR_CPU_INITTRACKER_H
