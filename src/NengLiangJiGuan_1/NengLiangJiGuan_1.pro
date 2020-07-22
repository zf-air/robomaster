TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += \
        buff.cpp \
        main.cpp
INCLUDEPATH+= E:\opencv\build\include\opencv\
            E:\opencv\build\include\opencv2\
            E:\opencv\build\include

LIBS += E:\opencv\QtBuild\lib\libopencv_*.a

HEADERS += \
    buff.h
