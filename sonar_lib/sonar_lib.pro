TARGET = sonar_lib
TEMPLATE = lib
VERSION = 0.1

CONFIG += c++11

CONFIG -= app_bundle
CONFIG -= qt

CONFIG += shared

android {
   LIBS += -L$$[QT_INSTALL_PLUGINS]/platforms/android
}

CONFIG(debug, debug|release) {
    OBJECTS_DIR = debug/obj
} else {
    OBJECTS_DIR = release/obj
}

INCLUDEPATH += $$PWD

windows  {
    QMAKE_CXXFLAGS += /bigobj
}

include (external/external_libs.pri)

include (General/General.pri)
include (DebugTools/DebugTools.pri)

HEADERS += \
    System.h \
    SourceFrame.h

SOURCES += \
    System.cpp \
    SourceFrame.cpp
