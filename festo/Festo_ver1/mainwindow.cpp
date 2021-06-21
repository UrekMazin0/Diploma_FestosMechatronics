#include "mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    this->showMaximized();
    scene = new QGraphicsScene(this);
    view = new QGraphicsView(this);
    view->setScene(scene);
    mainWidget = new QWidget(this);
    setCentralWidget(mainWidget);
    mainLayout = new QGridLayout(mainWidget);
    mainLayout->addWidget(view,0,0,10,10);
    toolBar = new QToolBar(this);
    toolBar->setIconSize(QSize(30,30));
    addToolBar(toolBar);
    toolBar->setStyleSheet ("QToolBar {background: rgb(30, 30, 30)}");
    toolBar->setMovable (false);
    actionNewFile = new QAction(QIcon(":/new.png"),tr("New"));
    actionNewFile->setShortcut(Qt::CTRL+Qt::Key_N);
    toolBar->addAction(actionNewFile);
    actionOpenFile = new QAction(QIcon(":/open.png"),tr("Open"));
    actionOpenFile->setShortcut(Qt::CTRL+Qt::Key_O);
    toolBar->addAction(actionOpenFile);
    actionSaveFile = new QAction(QIcon(":/save.png"),tr("Save"));
    actionSaveFile->setShortcut(Qt::CTRL+Qt::Key_S);
    toolBar->addAction(actionSaveFile);
    actionSaveFileAs = new QAction(QIcon(":/saveAs.png"),tr("SaveAs"));
    actionSaveFileAs->setShortcut(Qt::CTRL+Qt::Key_A);
    toolBar->addAction(actionSaveFileAs);
    actionRotation = new QAction(QIcon(":rotation.png"),tr("Rotation"));
//    actionRotation->setShortcut (Qt::Key);
    toolBar->addAction(actionRotation);
    actionStart = new QAction(QIcon(":start.png"),tr("Start"));
    actionStart->setShortcut (Qt::CTRL+Qt::Key_R);
    toolBar->addAction(actionStart);
    actionStop = new QAction(QIcon(":stop.png"),tr("Stop"));
    actionStop->setShortcut (Qt::CTRL+Qt::Key_T);
    toolBar->addAction(actionStop);;
    actionExit = new QAction(QIcon(":/close.png"),tr("Exit"));
    actionExit->setShortcut(Qt::CTRL+Qt::Key_X);
    toolBar->addAction(actionExit);
    connect(actionNewFile,SIGNAL(triggered(bool)),this,SLOT(newFile()));
    connect(actionOpenFile,SIGNAL(triggered(bool)),this,SLOT(openFile()));
    connect(actionSaveFile,SIGNAL(triggered(bool)),this,SLOT(saveFile()));
    connect(actionSaveFileAs,SIGNAL(triggered(bool)),this,SLOT(saveAs()));
    connect(actionExit,SIGNAL(triggered(bool)),this,SLOT(close()));
    connect (actionStart,SIGNAL(triggered(bool)),this,SLOT(start()));
    connect (actionStop, SIGNAL(triggered(bool)),this,SLOT(stop()));
    connect (actionRotation,SIGNAL(triggered(bool)),this,SLOT(rotation()));
    QMenu *menuFile = menuBar()->addMenu(tr("&File"));
    menuFile->addAction(actionNewFile);
    menuFile->addAction(actionOpenFile);
    menuFile->addAction(actionSaveFile);
    menuFile->addAction(actionSaveFileAs);
    menuFile->addAction(actionExit);
    QMenu *menuDebug = menuBar ()->addMenu (tr("&Debug"));
    menuDebug->addAction(actionStart);
    menuDebug->addAction(actionStop);
    QMenu *menuReference = menuBar()->addMenu (tr("&Reference"));
}

MainWindow::~MainWindow()
{

}

void MainWindow::newFile()
{

}
void MainWindow::openFile()
{

}
void MainWindow::saveFile()
{

}
void MainWindow::saveAs()
{

}

void MainWindow::start()
{

}
void MainWindow::stop()
{

}
void MainWindow::rotation()
{

}

