QT += qml quick widgets

TARGET = debug_gui

CONFIG += c++11
VERSION = 0.1

CONFIG -= app_bundle

CONFIG(debug, debug|release) {
    MOC_DIR = debug/moc
    RCC_DIR = debug/rcc
    UI_DIR = debug/ui
    OBJECTS_DIR = debug/obj

    windows {
        PRE_TARGETDEPS += $$OUT_PWD/../sonar/sonar.lib
    } else: linux {
        PRE_TARGETDEPS += $$OUT_PWD/../sonar/libsonar.a
    }
    LIBS += -L$$OUT_PWD/../sonar -lsonar
} else {
    MOC_DIR = release/moc
    RCC_DIR = release/rcc
    UI_DIR = release/ui
    OBJECTS_DIR = release/obj

    windows {
        PRE_TARGETDEPS += $$OUT_PWD/../sonar/sonar.lib
    } else: linux {
        PRE_TARGETDEPS += $$OUT_PWD/../sonar/libsonar.a
    }
    LIBS += -L$$OUT_PWD/../sonar -lsonar
}

linux {
    QMAKE_CXXFLAGS += -Wno-unused-function
}

include (../sonar/external/external_libs.pri)
include (../sonar/external/opencv.pri)

INCLUDEPATH += $$PWD/../sonar/include
DEPENDPATH += $$PWD/../sonar/include

SOURCES += \
    main.cpp \
    test_synthetic_initialization.cpp

HEADERS += \
    test_synthetic_initialization.h

DEFINES += _USE_MATH_DEFINES
