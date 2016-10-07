#-------------------------------------------------
#
# Project created by QtCreator 2016-06-22T15:44:24
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = sfm
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    lib/Utils.cpp \
    lib/masteringocv.cpp

HEADERS  += mainwindow.h \
    lib/Utils.h \
    lib/masteringocv.h

FORMS    += mainwindow.ui

RESOURCES += \
    resources.qrc

INCLUDEPATH += /usr/local/include/opencv2
INCLUDEPATH += /home/surf

CONFIG(debug,debug|release){
LIBS += -L/usr/local/lib
LIBS += -lopencv_core
LIBS += -lopencv_imgproc
LIBS += -lopencv_imgcodecs
LIBS += -lopencv_highgui
LIBS += -lopencv_ml
LIBS += -lopencv_video
LIBS += -lopencv_features2d
LIBS += -lopencv_calib3d
LIBS += -lopencv_objdetect
LIBS += -lopencv_contrib
LIBS += -lopencv_legacy
LIBS += -lopencv_flann
LIBS += -lopencv_imgcodecs
LIBS += -lopencv_sfm
LIBS += -lopencv_viz
LIBS += -lopencv_xfeatures2d
LIBS += -lopencv_videoio
}

CONFIG(release,debug|release){
LIBS += -L/usr/local/lib
LIBS += -lopencv_core
LIBS += -lopencv_imgproc
LIBS += -lopencv_imgcodecs
LIBS += -lopencv_highgui
LIBS += -lopencv_ml
LIBS += -lopencv_video
LIBS += -lopencv_features2d
LIBS += -lopencv_calib3d
LIBS += -lopencv_objdetect
LIBS += -lopencv_contrib
LIBS += -lopencv_legacy
LIBS += -lopencv_flann
LIBS += -lopencv_imgcodecs
LIBS += -lopencv_sfm
LIBS += -lopencv_viz
LIBS += -lopencv_xfeatures2d
LIBS += -lopencv_videoio
}
