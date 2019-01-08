/**
* This file is part of sonar library
* Copyright (C) 2018 Vlasov Aleksey ijonsilent53@gmail.com
* For more information see <https://github.com/DistinctVision/sonar>
**/

#ifndef SONAR_COMMON_H
#define SONAR_COMMON_H

#if defined(_WIN32) || defined(_WIN64)
#if defined(SONAR_SET_EXPORT)
#define SONAR_EXPORT __declspec(dllexport)
#else
#define SONAR_EXPORT __declspec(dllimport)
#endif
#else
#define SONAR_EXPORT
#endif

#endif // SONAR_COMMON_H
