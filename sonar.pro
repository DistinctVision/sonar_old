TEMPLATE = subdirs

CONFIG += ordered

DEFINES += BLABLA

SUBDIRS += \
    sonar \
    debug_gui

debug_gui.depends = sonar


