DEFINES += OPENCV_LIB
DEFINES += DEBUG_TOOLS_ENABLED

windows {
    TEST_OPENCV_INCLUDE_PATH = $$(OPENCV_INCLUDE_PATH)
    isEmpty(TEST_OPENCV_INCLUDE_PATH) {
        error("Variable \"OPENCV_INCLUDE_PATH\" is not set")
    } else {
        TEST_OPENCV_LIB_PATH = $$(OPENCV_LIB_PATH)
        isEmpty(TEST_OPENCV_LIB_PATH) {
            error("Variable \"OPENCV_LIB_PATH\" is not set")
        } else {
            INCLUDEPATH += $$(OPENCV_INCLUDE_PATH)
            DEPENDPATH += $$(OPENCV_INCLUDE_PATH)

            CONFIG(debug, debug|release) {
                OPENCV_VERSION=400d
            } else {
                OPENCV_VERSION=400
            }
            LIBS += -L$$(OPENCV_LIB_PATH) -lopencv_world$$OPENCV_VERSION
        }
    }
} else: unix {
    INCLUDEPATH += /usr/local/include/opencv4
    DEPENDPATH += /usr/local/include/opencv4

    LIBS += -L/usr/local/lib -lopencv_world
} else {
    error("OpenCV not included")
}
