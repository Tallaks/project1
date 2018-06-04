#-------------------------------------------------
#
# Project created by QtCreator 2017-02-08T14:41:36
#
#-------------------------------------------------

QT += core gui
QT += axserver opengl
QT += widgets
QT += printsupport

TARGET = 22222
TEMPLATE = app


SOURCES += main.cpp\
        estar.cpp \
    emath.cpp \
    count.cpp \
    j2000.cpp \
    dialogmotion.cpp \
    succes.cpp \
    qchartviewer.cpp \
    GraphIK.cpp \
    graphka.cpp \
    graphpr.cpp \
    graphicalmodel.cpp \
    modelwindow.cpp

HEADERS  += estar.h \
    emath.h \
    count.h \
    j2000.h \
    dialogmotion.h \
    succes.h \
    qchartviewer.h \
    traj.h \
    GraphIK.h \
    graphka.h \
    graphpr.h \
    graphicalmodel.h \
    modelwindow.h

RESOURCES += \
    realtimezoomscroll.qrc

DEFINES += CHARTDIR_HIDE_OBSOLETE _CRT_SECURE_NO_WARNINGS

win32:contains(QMAKE_HOST.arch, x86_64) {
  LIBS += ../../lib64/chartdir60.lib
  QMAKE_POST_LINK = copy /Y ..\\..\\lib64\\chartdir60.dll $(DESTDIR)
} else {
  LIBS += ../../lib32/chartdir60.lib
  QMAKE_POST_LINK = copy /Y ..\\..\\lib32\\chartdir60.dll $(DESTDIR)
}
