/**
* This file is part of sonar library
* Copyright (C) 2019 Vlasov Aleksey ijonsilent53@gmail.com
* For more information see <https://github.com/DistinctVision/sonar>
**/

#ifndef SONAR_TYPES_H
#define SONAR_TYPES_H

#if defined(OPENGV_LIB)

#include <opengv/types.hpp>

namespace sonar {

using point_t = opengv::point_t;
using points_t = opengv::points_t;
using bearingVector_t = opengv::bearingVector_t;
using bearingVectors_t = opengv::bearingVectors_t;
using translation_t = opengv::translation_t;
using essential_t = opengv::essential_t;
using essentials_t = opengv::essentials_t;
using transformation_t = opengv::transformation_t;
using transformations_t = opengv::transformations_t;

} // namespace sonar

#else

#include <vector>
#include <Eigen/Eigen>

namespace sonar {

using point_t = Eigen::Vector3d;
using points_t = std::vector<point_t, Eigen::aligned_allocator<point_t>>;
using bearingVector_t = Eigen::Vector3d;
using bearingVectors_t = std::vector<bearingVector_t, Eigen::aligned_allocator<bearingVector_t>>;
using translation_t = Eigen::Vector3d;
using essential_t = Eigen::Matrix3d;
using essentials_t = std::vector<essential_t, Eigen::aligned_allocator<essential_t>>;
using transformation_t = Eigen::Matrix<double, 3, 4>;
using transformations_t = std::vector<transformation_t, Eigen::aligned_allocator<transformation_t>>;

} // namespace sonar

#endif

#endif // SONAR_TYPES_H
