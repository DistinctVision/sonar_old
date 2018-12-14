DEFINES += OPENGV_LIB

linux {
    INCLUDEPATH += /usr/local/include/opengv
    DEPENDPATH += /usr/local/include/opengv

    LIBS += -L/usr/local/lib/ -lopengv
} else {
    error("OpenGV not included")
}
