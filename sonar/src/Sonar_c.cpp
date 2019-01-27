/**
* This file is part of sonar library
* Copyright (C) 2019 Vlasov Aleksey ijonsilent53@gmail.com
* For more information see <https://github.com/DistinctVision/sonar>
**/

#include "sonar/Sonar_c.h"

#include <vector>
#include <memory>
#include <cstring>

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

namespace sonar {

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
        m_system->reset();
    }

    void process_image_frame(const uchar * imageData, int imageWidth, int imageHeight)
    {
        m_system->process(SourceFrame(ConstImage<uchar>(Point2i(imageWidth, imageHeight), imageData, false)));
    }

    void get_current_frame_pose(double * out_rotation_ptr, double * out_translation_ptr)
    {
        Eigen::Matrix3d R;
        Eigen::Vector3d t;
        if (m_mapFrame)
        {
            R = m_mapFrame->rotation();
            t = m_mapFrame->translation();
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
    std::shared_ptr<MapFrame> m_mapFrame;
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

void sonar_get_current_frame_pose(double * out_rotation_ptr, double * out_translation_ptr)
{
    sonar::SystemContext::instance().get_current_frame_pose(out_rotation_ptr, out_translation_ptr);
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
