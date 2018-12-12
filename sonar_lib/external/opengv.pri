DEFINES += OPENGV_LIB

linux {
    INCLUDEPATH += /usr/local/include/opengv
    DEPENDPATH += /usr/local/include/opengv
} else {
    error("Opengv not included")
}
