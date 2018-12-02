DEFINES += MODULE_IMAGE_TOOLS

HEADERS += \
    $$PWD/FastCorner.h \
    $$PWD/FeatureDetector.h \
    $$PWD/OpticalFlow.h \
    $$PWD/OpticalFlowCalculator.h 

SOURCES += \
    $$PWD/FastCorner.cpp \
    $$PWD/faster_corner_10.cxx \
    $$PWD/FeatureDetector.cpp \
    $$PWD/OpticalFlow.cpp \
    $$PWD/OpticalFlowCalculator.cpp
