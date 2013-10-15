TEMPLATE = app
TARGET = triangulation

CONFIG += QtGui
QT += opengl

OBJECTS_DIR = bin

QMAKE_CXXFLAGS = -std=c++11 -Wall

macx {
    QMAKE_CXXFLAGS += -stdlib=libc++  
    QMAKE_LFLAGS += -lc++
}

CONFIG += precompile_header
PRECOMPILED_HEADER = stdafx.h

DEPENDPATH += src \
              visualization/headers \
              visualization/headers/common \
              visualization/headers/io \
              visualization/headers/visualization \

INCLUDEPATH += src \
               visualization/headers \

HEADERS += src/stdafx.h \
           src/viewer.h \
           src/triangulation.h \

SOURCES += src/main.cpp \
           src/triangulation.cpp \ 

LIBS += -Lvisualization -lvisualization
