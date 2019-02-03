#include "test_demo.h"

#include <memory>
#include <iostream>
#include <utility>

#include <Eigen/Core>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

#include <sonar/General/cast.h>
#include <sonar/General/Image.h>
#include <sonar/General/ImageUtils.h>
#include <sonar/types.h>
#include <sonar/SourceFrame.h>
#include <sonar/MapFrame.h>
#include <sonar/CameraTools/PinholeCamera.h>
#include <sonar/AbstractInitTracker.h>
#include <sonar/AbstractInitializator.h>
#include <sonar/System.h>

using namespace sonar;
using namespace std;
using namespace Eigen;

void draw_cube(cv::Mat cvFrame, const shared_ptr<const MapFrame> & mapFrame, double scale = 0.5)
{
    static const Vector3d vertices[8] = {
        Vector3d(-0.5, -0.5, 0.0),
        Vector3d(0.5, -0.5, 0.0),
        Vector3d(0.5, 0.5, 0.0),
        Vector3d(-0.5, 0.5, 0.0),
        Vector3d(-0.5, -0.5, 1.0),
        Vector3d(0.5, -0.5, 1.0),
        Vector3d(0.5, 0.5, 1.0),
        Vector3d(-0.5, 0.5, 1.0),
    };
    static const pair<int, int> edges[] = {
        make_pair(0, 1),
        make_pair(1, 2),
        make_pair(2, 3),
        make_pair(3, 0),
        make_pair(4, 5),
        make_pair(5, 6),
        make_pair(6, 7),
        make_pair(7, 4),
        make_pair(0, 4),
        make_pair(1, 5),
        make_pair(2, 6),
        make_pair(3, 7),
    };

    shared_ptr<const AbstractCamera> camera = mapFrame->camera();
    Matrix3d R = mapFrame->rotation();
    Vector3d t = mapFrame->translation();
    for (auto it_e = begin(edges); it_e != end(edges); ++it_e)
    {
        Vector3d v1 = R * vertices[it_e->first] * scale + t;
        Vector3d v2 = R * vertices[it_e->second] * scale + t;
        if ((cast<float>(v1.z()) < numeric_limits<float>::epsilon()) ||
                (cast<float>(v2.z()) < numeric_limits<float>::epsilon()))
        {
            continue;
        }
        cv::Point2i p1 = cv_cast<int>(camera->toImagePoint(v1));
        cv::Point2i p2 = cv_cast<int>(camera->toImagePoint(v2));
        cv::line(cvFrame, p1, p2, cv::Scalar(0, 255, 0), 2);
    }
}

void draw_points(cv::Mat cvFrame, const shared_ptr<const MapFrame> & mapFrame,
                 const points_t & points)
{
    shared_ptr<const AbstractCamera> camera = mapFrame->camera();
    Matrix3d R = mapFrame->rotation();
    Vector3d t = mapFrame->translation();
    for (auto it = points.cbegin(); it != points.cend(); ++it)
    {
        cv::Point2i p = cv_cast<int>(camera->toImagePoint(R * (*it) + t));
        cv::circle(cvFrame, p, 2, cv::Scalar(255, 0, 0), 1);
    }
}

void draw_grid(cv::Mat cvFrame, const shared_ptr<const MapFrame> & mapFrame)
{
    shared_ptr<const AbstractCamera> camera = mapFrame->camera();
    Matrix3d R = mapFrame->rotation();
    Vector3d t = mapFrame->translation();
    for (int i = 0; i <= 10; ++i)
    {
        Vector3d v1((i - 5) * 0.1, -0.5, 0.0);
        Vector3d v2((i - 5) * 0.1, 0.5, 0.0);

        cv::Point2i p1 = cv_cast<int>(camera->toImagePoint(R * v1 + t));
        cv::Point2i p2 = cv_cast<int>(camera->toImagePoint(R * v2 + t));

        cv::line(cvFrame, p1, p2, cv::Scalar(255, 0, 0), 2);
    }
    for (int i = 0; i <= 10; ++i)
    {
        Vector3d v1(-0.5, (i - 5) * 0.1, 0.0);
        Vector3d v2(0.5, (i - 5) * 0.1,  0.0);

        cv::Point2i p1 = cv_cast<int>(camera->toImagePoint(R * v1 + t));
        cv::Point2i p2 = cv_cast<int>(camera->toImagePoint(R * v2 + t));

        cv::line(cvFrame, p1, p2, cv::Scalar(255, 0, 0), 2);
    }
}

void draw_camera_model(cv::Mat cvFrame, const shared_ptr<const MapFrame> & mapFrame,
                       shared_ptr<const AbstractCamera> camera,
                       const Matrix3d & R, const Vector3d & t,
                       cv::Scalar color)
{
    static const pair<int, int> edges[8] = {
        make_pair(0, 1),
        make_pair(0, 2),
        make_pair(0, 3),
        make_pair(0, 4),
        make_pair(1, 2),
        make_pair(2, 3),
        make_pair(3, 4),
        make_pair(4, 1)
    };

    Matrix3d invR = R.inverse();
    Vector3d pos = - invR * t;

    Point2d image_size = cast<double>(camera->imageSize());

    Vector3d vertices[5] = {
        pos,
        R * camera->toLocalDir(Point2d(0.0, 0.0)).normalized() * 0.1 + pos,
        R * camera->toLocalDir(Point2d(image_size.x, 0.0)).normalized() * 0.1 + pos,
        R * camera->toLocalDir(image_size).normalized() * 0.1 + pos,
        R * camera->toLocalDir(Point2d(0.0, image_size.y)).normalized() * 0.1 + pos
    };

    Matrix3d frame_R = mapFrame->rotation();
    Vector3d frame_t = mapFrame->translation();

    Point2d local_points[5] = {
        mapFrame->camera()->toImagePoint(frame_R * vertices[0] + frame_t),
        mapFrame->camera()->toImagePoint(frame_R * vertices[1] + frame_t),
        mapFrame->camera()->toImagePoint(frame_R * vertices[2] + frame_t),
        mapFrame->camera()->toImagePoint(frame_R * vertices[3] + frame_t),
        mapFrame->camera()->toImagePoint(frame_R * vertices[4] + frame_t),
    };

    for (const pair<int, int> & edge : edges)
    {
        cv::line(cvFrame, cv_cast<float>(local_points[edge.first]),
                          cv_cast<float>(local_points[edge.second]), color, 2);
    }
}

bool test_demo()
{
    cv::VideoCapture capture;

    if (!capture.open(0))
    {
        cerr << "camera not found" << endl;
        return false;
    }

    if (!capture.isOpened())
    {
        cerr << "camera not open" << endl;
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

    double pixelFocalLength = frame_bw.width() * 1.0;
    auto camera = make_shared<PinholeCamera>(Point2d(pixelFocalLength, - pixelFocalLength),
                                             cast<double>(frame_bw.size()) * 0.5,
                                             frame_bw.size());
    System system;
    system.setCamera(camera);

    bool startFlag = false;
    bool stopFlag = false;
    while (capture.isOpened())
    {
        if (!stopFlag)
        {
            if (!capture.read(cvFrameImage))
            {
                break;
            }
        }
        if (startFlag)
        {
            frame_rgb = image_utils::convertCvMat_rgb_u(cvFrameImage);
            image_utils::convertToGrayscale(frame_bw, frame_rgb);

            SourceFrame sourceFrame(frame_bw);
            auto mapFrame = system.process(sourceFrame);

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
            else if (system.trackingState() == TrackingState::Tracking)
            {
                if (!stopFlag)
                {
                    stopFlag = true;

                    auto initInfo = system.initializator()->lastInitializationInfo();
                    draw_grid(cvFrameImage, mapFrame);
                    draw_points(cvFrameImage, mapFrame, initInfo.points);
                    draw_cube(cvFrameImage, mapFrame);
                    draw_camera_model(cvFrameImage, mapFrame,
                                      system.camera(),
                                      initInfo.firstTransfrom.block<3, 3>(0, 0), initInfo.firstTransfrom.col(3),
                                      cv::Scalar(0, 0, 255));
                    draw_camera_model(cvFrameImage, mapFrame,
                                      system.camera(),
                                      initInfo.secondTransfrom.block<3, 3>(0, 0), initInfo.secondTransfrom.col(3),
                                      cv::Scalar(155, 255, 0));
                }
            }
            else
            {
                startFlag = false;
            }
        }

        cv::imshow("frame", cvFrameImage);

        int key = cv::waitKey(33);
        if (key == 27)
        {
            break;
        }
        else if (key == 32)
        {
            if (!startFlag)
            {
                startFlag = true;
                system.start();
            }
            else if (stopFlag)
            {
                stopFlag = false;
                startFlag = false;
                system.reset();
            }
        }
    }

    return true;
}
