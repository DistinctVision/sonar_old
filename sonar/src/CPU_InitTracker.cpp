/**
* This file is part of sonar library
* Copyright (C) 2019 Vlasov Aleksey ijonsilent53@gmail.com
* For more information see <https://github.com/DistinctVision/sonar>
**/

#include <sonar/CPU_InitTracker.h>
#include <algorithm>

using namespace std;

namespace sonar {

CPU_InitTracker::CPU_InitTracker():
    AbstractInitTracker()
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

bool CPU_InitTracker::process(const SourceFrame & sourceFrame)
{
    ///TODO make lk filter

    assert(sourceFrame.sourceType() == SourceFrame::SourceType::Image);
    ConstImage<uchar> image = sourceFrame.image();
    ImagePyramid_u imagePyramid(image, m_opticalFlow.numberLevels());
    switch (indexStep()) {
    case 0: {
        vector<FeatureDetector::FeatureCorner> corners = m_featureDetector.detectCorners(imagePyramid);
        vector<Point2f> & features = m_capturedFramePoints[0];
        features.resize(corners.size());
        for (size_t i = 0; i < corners.size(); ++i)
            features[i] = cast<float>(corners[i].pos);
        m_capturedFramePoints[1] = features;
        m_opticalFlow.setFirstPyramid(imagePyramid.copy());
        _capture(sourceFrame);
    } return true;
    case 1: {
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
        return false;
    }
    else
    {
        float step = _medianDistance(m_capturedFramePoints[indexStep() - 1],
                                     m_capturedFramePoints[indexStep()]);
        if (step > medianDistanceStep())
        {
            _capture(sourceFrame);
            if (indexStep() >= 3)
                return true;
            m_capturedFramePoints[indexStep()] = m_capturedFramePoints[indexStep() - 1];
            m_opticalFlow.setFirstPyramid(imagePyramid.copy());
        }
    }
    return true;
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
