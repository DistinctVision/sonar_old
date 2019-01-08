#include "test_demo.h"
#include <memory>
#include <iostream>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

#include <sonar/General/cast.h>
#include <sonar/General/Image.h>
#include <sonar/General/ImageUtils.h>
#include <sonar/SourceFrame.h>
#include <sonar/MapFrame.h>
#include <sonar/CameraTools/PinholeCamera.h>
#include <sonar/AbstractInitTracker.h>
#include <sonar/System.h>

using namespace sonar;
using namespace std;

bool test_demo()
{
    cv::VideoCapture capture(0);

    if (capture.isOpened())
    {
        cerr << "camera not found" << endl;
        return false;
    }

    cv::Mat cvFrameImage;
    if (!capture.read(cvFrameImage))
    {
        cerr << "frame not readed" << endl;
        return false;
    }
    Image<Rgb_u> frame_rgb = image_utils::convertCvMat_rgb_u(cvFrameImage);
    Image<uchar> frame_bw = image_utils::convertToGrayscale(frame_rgb);

    double focalLength = 1.2;
    auto camera = make_shared<PinholeCamera>(Point2d(frame_bw.width() * focalLength, frame_bw.height() * focalLength),
                                             cast<double>(frame_bw.size()) * 0.5,
                                             frame_bw.size());
    System system;
    system.setCamera(camera);

    bool startedFlag = false;
    while (capture.isOpened())
    {
        if (!capture.read(cvFrameImage))
        {
            break;
        }
        if (startedFlag)
        {
            frame_rgb = image_utils::convertCvMat_rgb_u(cvFrameImage);
            image_utils::convertToGrayscale(frame_bw, frame_rgb);

            SourceFrame sourceFrame(frame_bw);
            system.process(sourceFrame);

            if (system.trackingState() == TrackingState::Initializing)
            {
                auto initTracker = system.initTracker();
                for (int step = 1; step <= initTracker->indexStep(); ++step)
                {
                    vector<Point2f> prevPoints = initTracker->capturedFramePoints(step - 1);
                    vector<Point2f> curPoints = initTracker->capturedFramePoints(step);

                    for (size_t i = 0; i < curPoints.size(); ++i)
                    {
                        cv::line(cvFrameImage,
                                 cv_cast<float>(prevPoints[i]), cv_cast<float>(curPoints[i]),
                                 cv::Scalar(255, 0, 0), 2);
                    }
                }
            }
        }

        cv::imshow("frame", cvFrameImage);

        int key = cv::waitKey(33);
        if (key == 27)
            break;
        if (!startedFlag)
        {
            if (key == 32)
            {
                startedFlag = true;
                system.start();
            }
        }
    }

    return true;
}
