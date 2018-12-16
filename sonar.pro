TEMPLATE = subdirs

CONFIG += ordered

SUBDIRS += \
    sonar \
    debug_gui

debug_gui.depends = sonar


