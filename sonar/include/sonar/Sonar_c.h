/**
* This file is part of sonar library
* Copyright (C) 2019 Vlasov Aleksey ijonsilent53@gmail.com
* For more information see <https://github.com/DistinctVision/sonar>
**/

#ifndef SONAR_C_H
#define SONAR_C_H

#if defined(_WIN32) || defined(_WIN64)
#if defined(SONAR_SET_EXPORT)
#define SONAR_EXPORT __declspec(dllexport)
#else
#define SONAR_EXPORT __declspec(dllimport)
#endif
#else
#define SONAR_EXPORT
#endif

extern "C" {

SONAR_EXPORT void sonar_setPinholeCamera(int imageWidth, int imageHeight,
                                         double focalLength_x, double focalLength_y,
                                         double opticalCenter_x, double opticalCenter_y);

SONAR_EXPORT int sonar_getTrackingState();

SONAR_EXPORT void sonar_start();

SONAR_EXPORT void sonar_reset();

SONAR_EXPORT void sonar_process_image_frame(const unsigned char * imageData, int imageWidth, int imageHeight);

SONAR_EXPORT void sonar_get_current_frame_pose(double * out_rotation_ptr, double * out_translation_ptr);

SONAR_EXPORT int sonar_get_number_init_image_points(int indexStep);

SONAR_EXPORT void sonar_get_init_image_points(float * out_data_points, int indexStep);

} // extern "C"

#endif // SONAR_C_H
