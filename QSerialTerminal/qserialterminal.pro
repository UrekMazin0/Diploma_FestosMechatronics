QT += widgets serialport
requires(qtConfig(combobox))

TARGET = qserialterminal
TEMPLATE = app

SOURCES += \
    infoClasses.cpp \
    main.cpp \
    mainwindow.cpp \
    qserialprotocol.cpp \
    settingsdialog.cpp \
    console.cpp

HEADERS += \
    infoClasses.h \
    mainwindow.h \
    qserialprotocol.h \
    settingsdialog.h \
    console.h

FORMS += \
    mainwindow.ui \
    settingsdialog.ui

RESOURCES += \
    terminal.qrc

INSTALLS += target

DISTFILES +=
