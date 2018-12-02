DEFINES += EIGEN3_LIB

linux {
    INCLUDEPATH += /usr/local/include/eigen3
    DEPENDPATH += /usr/local/include/eigen3
} else {
    error("Eigen not included")
}
