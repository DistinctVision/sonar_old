include (DebugTools/DebugTools.pri)
include (ImageTools/ImageTools.pri)
include (CameraTools/CameraTools.pri)

SOURCES += \
    $$PWD/AbstractInitTracker.cpp \
    $$PWD/CPU_InitTracker.cpp \
    $$PWD/Initializator.cpp \
    $$PWD/MapFrame.cpp \
    $$PWD/PlaneFinder.cpp \
    $$PWD/SourceFrame.cpp \
    $$PWD/System.cpp
