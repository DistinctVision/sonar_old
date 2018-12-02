#include "CPU_InitTracker.h"
#include <algorithm>

using namespace std;

namespace sonar {

CPU_InitTracker::CPU_InitTracker():
    AbstractInitTracker(),
    m_indexStep(0)
{
}

const FeatureDetector & CPU_InitTracker::featureTracker() const
{
    return m_featureDetector;
}

FeatureDetector & CPU_InitTracker::featureTracker()
{
    return m_featureDetector;
}

const OpticalFlow & CPU_InitTracker::opticalFlow() const
{
    return m_opticalFlow;
}

OpticalFlow & CPU_InitTracker::opticalFlow()
{
    return m_opticalFlow;
}

void CPU_InitTracker::reset()
{
    m_indexStep = 0;
    for (int i = 0; i < 3; ++i)
        m_capturedFramePoints[i].clear();
}

bool CPU_InitTracker::process(const SourceFrame & sourceFrame)
{
    assert(sourceFrame.type() == SourceFrame::Type::Image);
    ConstImage<uchar> image = sourceFrame.image();
    switch (m_indexStep) {
    case 0: {
        ImagePyramid_u imagePyramid(image, m_opticalFlow.numberLevels());
        vector<FeatureDetector::FeatureCorner> corners = m_featureDetector.detectCorners(imagePyramid);
        vector<Point2f> & features = m_capturedFramePoints[0];
        features.resize(corners.size());
        for (size_t i = 0; i < corners.size(); ++i)
            features[i] = cast<float>(corners[i].pos);
        m_capturedFramePoints[1] = features;
        m_opticalFlow.setFirstPyramid(imagePyramid);
        ++m_indexStep;
    } break;
    case 1: {
        ImagePyramid_u imagePyramid(image, m_opticalFlow.numberLevels());
        m_opticalFlow.setSecondPyramid(imagePyramid);
        vector<TrackingResult> status;
        m_opticalFlow.tracking2dLK(status, m_capturedFramePoints[1], m_capturedFramePoints[0]);
        size_t cSize = 0;
        for (size_t i = 0; i < status.size(); ++i)
        {
            if (status[i] == TrackingResult::Fail)
                continue;
            for (int j = 0; j < 2; ++j)
                m_capturedFramePoints[j][cSize] = m_capturedFramePoints[j][i];
            ++cSize;
        }
        for (int j = 0; j < 2; ++j)
            m_capturedFramePoints[j].resize(cSize);
    } break;
    case 2: {
        ImagePyramid_u imagePyramid(image, m_opticalFlow.numberLevels());
        m_opticalFlow.setSecondPyramid(imagePyramid);
        vector<TrackingResult> status;
        m_opticalFlow.tracking2dLK(status, m_capturedFramePoints[2], m_capturedFramePoints[0]);
        size_t cSize = 0;
        for (size_t i = 0; i < status.size(); ++i)
        {
            if (status[i] == TrackingResult::Fail)
                continue;
            for (int j = 0; j < 3; ++j)
                m_capturedFramePoints[j][cSize] = m_capturedFramePoints[j][i];
            ++cSize;
        }
        for (int j = 0; j < 3; ++j)
            m_capturedFramePoints[j].resize(cSize);
    } break;
    default:
        return true;
    }
    if (cast<int>(m_capturedFramePoints[0].size()) < minNumberFeatures())
    {
        reset();
    }
    else
    {
        float step = _medianDistance(m_capturedFramePoints[m_indexStep - 1],
                                     m_capturedFramePoints[m_indexStep]);
        if (step > medianDistanceStep())
        {
            ++m_indexStep;
            if (m_indexStep >= 3)
                return true;
            m_capturedFramePoints[m_indexStep] = m_capturedFramePoints[m_indexStep - 1];
        }
    }
    return false;
}

bool CPU_InitTracker::isFinished() const
{
    return (m_indexStep == 3);
}

tuple<vector<Point2f>, vector<Point2f>, vector<Point2f>> CPU_InitTracker::getMotion() const
{
    return make_tuple(m_capturedFramePoints[0], m_capturedFramePoints[1], m_capturedFramePoints[2]);
}

float CPU_InitTracker::_medianDistance(const vector<Point2f> &pointsA,
                                       const vector<Point2f> &pointsB) const
{
    assert(pointsA.size() == pointsB.size());
    assert(!pointsA.empty());
    vector<float> distances_sq(pointsA.size());
    for (size_t i = 0; i < pointsA.size(); ++i)
        distances_sq[i] = (pointsB[i] - pointsA[i]).lengthSquared();
    nth_element(distances_sq.begin(), distances_sq.begin() + distances_sq.size() / 2, distances_sq.end());
    return sqrt(distances_sq[distances_sq.size() / 2]);
}

} // namespace sonar
