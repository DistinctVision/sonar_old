/**
* This file is part of sonar library
* Copyright (C) 2019 Vlasov Aleksey ijonsilent53@gmail.com
* For more information see <https://github.com/DistinctVision/sonar>
**/

#include "sonar/Sonar_c.h"

#include <vector>
#include <memory>
#include <cstring>

#include <fstream>

#include <Eigen/Eigen>

#include "sonar/General/cast.h"
#include "sonar/General/Point2.h"
#include "sonar/General/Image.h"
#include "sonar/CameraTools/AbstractCamera.h"
#include "sonar/CameraTools/PinholeCamera.h"
#include "sonar/SourceFrame.h"
#include "sonar/MapFrame.h"
#include "sonar/AbstractInitTracker.h"
#include "sonar/AbstractInitializator.h"
#include "sonar/System.h"

/*#include <sonar/DebugTools/debug_tools.h>
#include <opencv2/opencv.hpp>
#include "sonar/General/ImageUtils.h"*/

namespace sonar {

/*void draw_cube(cv::Mat cvFrame, const std::shared_ptr<const MapFrame> & mapFrame, double scale = 0.5)
{
    using namespace std;
    using namespace Eigen;

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
        if ((v1.z() < 0.0) || (v2.z() < 0.0))
            continue;
        cv::Point2i p1 = cv_cast<int>(camera->toImagePoint(v1));
        cv::Point2i p2 = cv_cast<int>(camera->toImagePoint(v2));
        cv::line(cvFrame, p1, p2, cv::Scalar(0, 255, 0), 2);
    }
}

void draw_grid(cv::Mat cvFrame, const std::shared_ptr<const MapFrame> & mapFrame)
{
    using namespace std;
    using namespace Eigen;

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
}*/

class SystemContext
{
public:
    static SystemContext & instance()
    {
        static SystemContext context;
        return context;
    }

    SystemContext():
        m_system(new System())
    {
    }

    void setPinholeCamera(int imageWidth, int imageHeight,
                          double focalLength_x, double focalLength_y,
                          double opticalCenter_x, double opticalCenter_y)
    {
        m_camera = std::make_shared<PinholeCamera>(Point2d(focalLength_x, focalLength_y),
                                                   Point2d(opticalCenter_x, opticalCenter_y),
                                                   Point2i(imageWidth, imageHeight));
        m_system->setCamera(m_camera);
    }

    int getTrackingState() const
    {
        return static_cast<int>(m_system->trackingState());
    }

    void start()
    {
        m_system->start();
    }

    void reset()
    {
        m_currentMapFrame.reset();
        m_system->reset();
    }

    void process_image_frame(const uchar * imageData, int imageWidth, int imageHeight)
    {
        m_currentMapFrame = m_system->process(SourceFrame(ConstImage<uchar>(Point2i(imageWidth, imageHeight), imageData, false)));
        /*if (m_currentMapFrame)
        {
            Image<Rgb_u> image = m_currentMapFrame->image().convert<Rgb_u>([](const Point2i &p, const uchar &v) { return Rgb_u(v, v, v); });
            cv::Mat cvImage = image_utils::convertToCvMat(image);
            draw_grid(cvImage, m_currentMapFrame);
            draw_cube(cvImage, m_currentMapFrame);
            cv::imshow("image", cvImage);
            debug::waitKey(33);
        }*/
    }

    void get_current_frame_pose_Rt(double * out_rotation_ptr, double * out_translation_ptr)
    {
        Eigen::Matrix3d R;
        Eigen::Vector3d t;
        if (m_currentMapFrame)
        {
            R = m_currentMapFrame->rotation();
            t = m_currentMapFrame->translation();
        }
        else
        {
            R.setIdentity();
            t.setZero();
        }
        out_rotation_ptr[0] = R(0, 0);
        out_rotation_ptr[1] = R(0, 1);
        out_rotation_ptr[2] = R(0, 2);
        out_rotation_ptr[3] = R(1, 0);
        out_rotation_ptr[4] = R(1, 1);
        out_rotation_ptr[5] = R(1, 2);
        out_rotation_ptr[6] = R(2, 0);
        out_rotation_ptr[7] = R(2, 1);
        out_rotation_ptr[8] = R(2, 2);
        out_translation_ptr[0] = t(0);
        out_translation_ptr[1] = t(1);
        out_translation_ptr[2] = t(2);
    }

    void get_current_frame_pose_qt(double * out_quaternion_ptr, double * out_translation_ptr)
    {
        Eigen::Quaterniond q;
        Eigen::Vector3d t;
        if (m_currentMapFrame)
        {
            q = Eigen::Quaterniond(m_currentMapFrame->rotation()).normalized();
            t = m_currentMapFrame->translation();
        }
        else
        {
            q.setIdentity();
            t.setZero();
        }

        out_quaternion_ptr[0] = q.x();
        out_quaternion_ptr[1] = q.y();
        out_quaternion_ptr[2] = q.z();
        out_quaternion_ptr[3] = q.w();
        out_translation_ptr[0] = t(0);
        out_translation_ptr[1] = t(1);
        out_translation_ptr[2] = t(2);
    }

    int get_number_init_image_points(int indexStep)
    {
        return cast<int>(m_system->initTracker()->capturedFramePoints(indexStep).size());
    }

    void get_init_image_points(float * out_data_points, int indexStep)
    {
        std::vector<Point2f> image_points = m_system->initTracker()->capturedFramePoints(indexStep);
        std::memcpy(out_data_points, image_points.data(), sizeof(Point2f) * image_points.size());
    }

private:
    std::unique_ptr<System> m_system;
    std::shared_ptr<AbstractCamera> m_camera;
    std::shared_ptr<const MapFrame> m_currentMapFrame;
};

} // namespace sonar

extern "C" {

void sonar_setPinholeCamera(int imageWidth, int imageHeight,
                            double focalLength_x, double focalLength_y,
                            double opticalCenter_x, double opticalCenter_y)
{
    sonar::SystemContext::instance().setPinholeCamera(imageWidth, imageHeight,
                                                      focalLength_x, focalLength_y,
                                                      opticalCenter_x, opticalCenter_y);
}

int sonar_getTrackingState()
{
    return sonar::SystemContext::instance().getTrackingState();
}

void sonar_start()
{
    sonar::SystemContext::instance().start();
}

void sonar_reset()
{
    sonar::SystemContext::instance().reset();
}

void sonar_process_image_frame(const unsigned char *imageData, int imageWidth, int imageHeight)
{
    sonar::SystemContext::instance().process_image_frame(imageData, imageWidth, imageHeight);
}

void sonar_get_current_frame_pose_Rt(double * out_rotation_ptr, double * out_translation_ptr)
{
    sonar::SystemContext::instance().get_current_frame_pose_Rt(out_rotation_ptr, out_translation_ptr);
}

void sonar_get_current_frame_pose_qt(double * out_quaternion_ptr, double * out_translation_ptr)
{
    sonar::SystemContext::instance().get_current_frame_pose_qt(out_quaternion_ptr, out_translation_ptr);
}

int sonar_get_number_init_image_points(int indexStep)
{
    return sonar::SystemContext::instance().get_number_init_image_points(indexStep);
}

void sonar_get_init_image_points(float * out_data_points, int indexStep)
{
    sonar::SystemContext::instance().get_init_image_points(out_data_points, indexStep);
}

} // extern "C"
