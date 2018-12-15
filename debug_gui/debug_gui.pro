QT += qml quick widgets

TARGET = qmlgui

CONFIG += c++11
VERSION = 0.1

CONFIG -= app_bundle

include (../sonar_lib/external/external_libs.pri)
include (../sonar_lib/external/opencv.pri)

CONFIG(debug, debug|release) {
    MOC_DIR = debug/moc
    RCC_DIR = debug/rcc
    UI_DIR = debug/ui
    OBJECTS_DIR = debug/obj

    LIBS += -L$$PWD/../qbuild/debug/ -lsonar_lib
} else {
    MOC_DIR = release/moc
    RCC_DIR = release/rcc
    UI_DIR = release/ui
    OBJECTS_DIR = release/obj

    LIBS += -L$$PWD/../qbuild/release/ -lsonar_lib
}

QMAKE_CXXFLAGS += -Wno-unused-function

INCLUDEPATH += $$PWD/../sonar_lib
DEPENDPATH += $$PWD/../sonar_lib

SOURCES += \
    main.cpp \
    test_synthetic_initialization.cpp

HEADERS += \
    test_synthetic_initialization.h
