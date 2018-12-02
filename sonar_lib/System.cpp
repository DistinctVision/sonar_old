#include "System.h"
#include "SourceFrame.h"
#include "AbstractInitTracker.h"
#include "CPU_InitTracker.h"
#include "Initializator.h"

#include <exception>

using namespace std;

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

void System::reset()
{
    if (m_initTracker)
        m_initTracker->reset();
    m_trackingState = TrackingState::NotStarted;
    m_trackingQuality = TrackingQuality::Ugly;
}

void System::process(const SourceFrame & sourceFrame)
{
    switch (m_trackingState) {
    case TrackingState::NotStarted:
    case TrackingState::Initializing: {
        m_initTracker->process(sourceFrame);
        if (m_initTracker->isFinished())
        {
            
        }
    } break;
    case TrackingState::Tracking: {
        m_trackingState = TrackingState::Lost;
        m_trackingQuality = TrackingQuality::Ugly;
    } break;
    default:
        break;
    }
}

} // namespace sonar
