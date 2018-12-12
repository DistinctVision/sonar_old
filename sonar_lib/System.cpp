#include "System.h"
#include "SourceFrame.h"
#include "AbstractInitTracker.h"
#include "CPU_InitTracker.h"
#include "Initializator.h"

#include <Eigen/Eigen>

#include <General/cast.h>

#include <exception>

using namespace std;
using namespace Eigen;

namespace sonar {

System::System():
    m_trackingState(TrackingState::NotStarted),
    m_trackingQuality(TrackingQuality::Ugly)
{
    m_initializator = make_shared<Initializator>();
    setProcessingMode(ProcessingMode::CPU);
}

TrackingState System::trackingState() const
{
    return m_trackingState;
}

TrackingQuality System::trackingQuality() const
{
    return m_trackingQuality;
}

ProcessingMode System::processingMode() const
{
    return m_processingMode;
}

void System::setProcessingMode(ProcessingMode mode)
{
    m_initTracker.reset();
    switch (mode) {
    case ProcessingMode::CPU:
        m_initTracker = make_shared<CPU_InitTracker>();
        break;
    default:
        throw invalid_argument("invalid processing mode");
    }
    m_processingMode = mode;
}

shared_ptr<const AbstractInitTracker> System::initTracker() const
{
    return m_initTracker;
}

shared_ptr<AbstractInitTracker> System::initTracker()
{
    return m_initTracker;
}

void System::start()
{
    if (m_trackingState == TrackingState::NotStarted)
        m_trackingState = TrackingState::Initializing;
}

void System::reset()
{
    if (m_initTracker)
        m_initTracker->reset();
    m_trackingState = TrackingState::NotStarted;
    m_trackingQuality = TrackingQuality::Ugly;
}

shared_ptr<const MapFrame> System::process(const SourceFrame & sourceFrame)
{
    switch (m_trackingState) {

    case TrackingState::NotStarted:
        break;

    case TrackingState::Initializing: {
        m_initTracker->process(sourceFrame);
        if (m_initTracker->isFinished())
        {
            bool successFlag;
            Initializator::Info initInfo;
            tie(successFlag, initInfo) = m_initializator->compute(
                        cast<double>(m_initTracker->capturedFramePoints(0)),
                        cast<double>(m_initTracker->capturedFramePoints(1)),
                        cast<double>(m_initTracker->capturedFramePoints(2)));
            if (!successFlag)
            {
                m_initTracker->reset();
                break;
            }
            Matrix3d R = initInfo.thirdWorldRotation.inverse();
            Vector3d t = - R * initInfo.thirdWorldPosition;
            return make_shared<MapFrame>(m_initTracker->capturedFrame(2), R, t);
        }
    } break;

    case TrackingState::Tracking: {
        m_trackingQuality = TrackingQuality::Good;
        Initializator::Info initInfo = m_initializator->lastInitializationInfo();
        Matrix3d R = initInfo.thirdWorldRotation.inverse();
        Vector3d t = - R * initInfo.thirdWorldPosition;
        return make_shared<MapFrame>(make_shared<SourceFrame>(sourceFrame), R, t);
    }

    default:
        break;
    }
    return shared_ptr<MapFrame>();
}

} // namespace sonar
