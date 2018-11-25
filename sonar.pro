TEMPLATE = subdirs

CONFIG += ordered

SUBDIRS += \
    sonar_lib \
    debug_gui

qmlgui.depends =  sonar_lib


