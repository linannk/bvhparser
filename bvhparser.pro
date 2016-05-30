# Don't use Qt library
QT -= core
QT -= gui

# Application
TEMPLATE = app

# Enable C++2011 features
CONFIG += c++11

# Target dir
DESTDIR = $$PWD/bin

# config debug and relase tmp dirs and target names
CONFIG(debug,debug|release){
    win32{
        contains(QMAKE_TARGET.arch, x86){
            TARGET = bvhparser[x86_dbg]
            OBJECTS_DIR = obj_tmp[x86_dbg]
            MOC_DIR = moc_tmp[x86_dbg]
            RCC_DIR = rcc_tmp[x86_dbg]
            UI_DIR = ui_tmp[x86_dbg]
        }
        contains(QMAKE_TARGET.arch, x86_64){
            TARGET = bvhparser[x64_dbg]
            OBJECTS_DIR = obj_tmp[x64_dbg]
            MOC_DIR = moc_tmp[x64_dbg]
            RCC_DIR = rcc_tmp[x64_dbg]
            UI_DIR = ui_tmp[x64_dbg]
        }
    } esle {
        TARGET = bvhparser[dbg]
        OBJECTS_DIR = obj_tmp[dbg]
        MOC_DIR = moc_tmp[dbg]
        RCC_DIR = rcc_tmp[dbg]
        UI_DIR = ui_tmp[dbg]
    }
}else{
    win32{
        contains(QMAKE_TARGET.arch, x86){
            TARGET = bvhparser[x86_rel]
            OBJECTS_DIR = obj_tmp[x86_rel]
            MOC_DIR = moc_tmp[x86_rel]
            RCC_DIR = rcc_tmp[x86_rel]
            UI_DIR = ui_tmp[x86_rel]
        }
        contains(QMAKE_TARGET.arch, x86_64){
            TARGET = bvhparser[x64_rel]
            OBJECTS_DIR = obj_tmp[x64_rel]
            MOC_DIR = moc_tmp[x64_rel]
            RCC_DIR = rcc_tmp[x64_rel]
            UI_DIR = ui_tmp[x64_rel]
        }
    } esle {
        TARGET = bvhparser[rel]
        OBJECTS_DIR = obj_tmp[rel]
        MOC_DIR = moc_tmp[rel]
        RCC_DIR = rcc_tmp[rel]
        UI_DIR = ui_tmp[rel]
    }
}

HEADERS += \
    bvh.h

SOURCES += \
    bvh.cpp \
    main.cpp

