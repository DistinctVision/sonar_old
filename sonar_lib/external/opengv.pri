DEFINES += OPENGV_LIB

linux {
    TEST_VARIABLE_OPENGV_DIR = $$(OPENGV_DIR)
    isEmpty(TEST_VARIABLE_OPENGV_DIR) {
        error("\"OPENGV_DIR\" is not set")
    } else {
        INCLUDEPATH += $$(OPENGV_DIR)/include
        LIBS += -L$$(OPENGV_DIR)/build/lib -lopengv
    }
} else {
    error("opengv library not included")
}
