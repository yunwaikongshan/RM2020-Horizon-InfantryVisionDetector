TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

LIBS +=/usr/local/lib/libopencv*.so
LIBS += -lpthread
LIBS += -pthread
LIBS += -lgxiapi \

SOURCES += main.cpp \
    BuffDetector/BuffAngleSolver.cpp \
    BuffDetector/FindBuff.cpp \
    Filter/Filter.cpp \
    FindArmor/AngleSolver.cpp \
    FindArmor/ImageProcess.cpp \
    FindArmor/LongFindArmor.cpp \
    FindArmor/ShootTuoluo.cpp \
    GetNum/GetNum.cpp \
    GetNum/svm.cpp \
    SendRecive/CRC_Check.cpp \
    SendRecive/RemoteController.cpp \
    SendRecive/serial.cpp \
    video/DaHengCamera.cpp \
    video/VideoCapture.cpp \
    DrawCurve.cpp \
    Variables.cpp

HEADERS += \
    include/AngleSolver.h \
    include/BuffAngleSolver.h \
    include/CRC_Check.h \
    include/DaHengCamera.h \
    include/DrawCurve.h \
    include/DxImageProc.h \
    include/Filter.h \
    include/FindBuff.h \
    include/GetNum.h \
    include/GxIAPI.h \
    include/ImageProcess.h \
    include/LongFindArmor.h \
    include/RemoteController.h \
    include/serial.h \
    include/ShootTuoluo.h \
    include/svm.h \
    include/Variables.h \
    include/VideoCapture.h
