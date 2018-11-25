QT += qml quick widgets

TARGET = qmlgui

CONFIG += c++11
VERSION = 0.1

CONFIG -= app_bundle

CONFIG(debug, debug|release) {
    MOC_DIR = debug/moc
    RCC_DIR = debug/rcc
    UI_DIR = debug/ui
    OBJECTS_DIR = debug/obj
} else {
    MOC_DIR = release/moc
    RCC_DIR = release/rcc
    UI_DIR = release/ui
    OBJECTS_DIR = release/obj
}
