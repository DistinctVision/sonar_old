/**
* This file is part of sonar library
* Copyright (C) 2019 Vlasov Aleksey ijonsilent53@gmail.com
* For more information see <https://github.com/DistinctVision/sonar>
**/

#include <sonar/System.h>
#include <sonar/SourceFrame.h>
#include <sonar/AbstractInitTracker.h>
#include <sonar/CPU_InitTracker.h>
#include <sonar/AbstractInitializator.h>
#include <sonar/HomographyInitializator.h>
#include <sonar/Initializator.h>
#include <sonar/MapFrame.h>

#include <exception>

#include <Eigen/Eigen>

#include <sonar/General/cast.h>

using namespace std;
using namespace Eigen;

namespace sonar {

System::System():
    m_trackingState(TrackingState::NotStarted),
    m_trackingQuality(TrackingQuality::Ugly)
{
    setInitializatorType(InitializatorType::Homography);
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

InitializatorType System::initializatorType() const
{
    return m_initializatorType;
}

void System::setInitializatorType(InitializatorType initializatorType)
{
    switch (m_initializatorType) {
    case InitializatorType::Homography:
        m_initializator = make_shared<HomographyInitializator>();
        break;
#if defined(OPENGV_LIB)
    case InitializatorType::Common:
        m_initializator = make_shared<Initializator>();
        break;
#endif
    default:
        throw std::invalid_argument("initializatorType is not valid");
    }
    m_initializatorType = initializatorType;
}

shared_ptr<const AbstractInitializator> System::initializator() const
{
    return m_initializator;
}

std::shared_ptr<AbstractInitializator> System::initializator()
{
    return m_initializator;
}

shared_ptr<const AbstractCamera> System::camera() const
{
    return m_camera;
}

void System::setCamera(const shared_ptr<const AbstractCamera> & camera)
{
    m_camera = camera;
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
    assert(m_camera);

    switch (m_trackingState) {

    case TrackingState::NotStarted:
        break;

    case TrackingState::Initializing: {
        m_initTracker->process(sourceFrame);
        if (m_initTracker->isFinished())
        {
            bool successFlag;
            AbstractInitializator::Info initInfo;
            tie(successFlag, initInfo) = m_initializator->compute(
                        m_camera,
                        cast<double>(m_initTracker->capturedFramePoints(0)),
                        m_camera,
                        cast<double>(m_initTracker->capturedFramePoints(1)),
                        m_camera,
                        cast<double>(m_initTracker->capturedFramePoints(2)));
            if (!successFlag)
            {
                m_initTracker->reset();
                break;
            }
            m_trackingState = TrackingState::Tracking;
            Matrix3d R = initInfo.thirdTransfrom.block<3, 3>(0, 0);
            Vector3d t = initInfo.thirdTransfrom.col(3);
            return make_shared<MapFrame>(m_initTracker->capturedFrame(2),
                                         m_camera, R, t);
        }
    } break;

    case TrackingState::Tracking: {
        m_trackingQuality = TrackingQuality::Good;
        AbstractInitializator::Info initInfo = m_initializator->lastInitializationInfo();
        Matrix3d R = initInfo.thirdTransfrom.block<3, 3>(0, 0);
        Vector3d t = initInfo.thirdTransfrom.col(3);
        return make_shared<MapFrame>(make_shared<SourceFrame>(sourceFrame),
                                     m_camera, R, t);
    }

    default:
        break;
    }
    return shared_ptr<MapFrame>();
}

} // namespace sonar
