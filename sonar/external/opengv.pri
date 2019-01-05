DEFINES += OPENGV_LIB

linux {
    INCLUDEPATH += /usr/local/include/opengv
    DEPENDPATH += /usr/local/include/opengv

    CONFIG(debug, debug|release) {
        #PRE_TARGETDEPS += /usr/local/lib/libopengvd.a
        LIBS += -L/usr/local/lib -lopengv
    } else {
        #PRE_TARGETDEPS += /usr/local/lib/libopengv.a
        LIBS += -L/usr/local/lib -lopengv
    }
} else: windows {
    TEST_OPENGV_DIR = $$(OPENGV_DIR)
    isEmpty(TEST_OPENGV_DIR) {
        error("Variable \"OPENGV_DIR\" is not set")
    } else {
        INCLUDEPATH += $$(OPENGV_DIR)\include
        DEPENDPATH += $$(OPENGV_DIR)\include

        CONFIG(debug, debug|release) {
            LIBS += -L$$(OPENGV_DIR)\lib -lopengvd
        } else {
            LIBS += -L$$(OPENGV_DIR)\lib -lopengv
        }
    }
} else {
    error("OpenGV not included")
}
