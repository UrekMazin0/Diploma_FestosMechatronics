#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QWidget>
#include <QLabel>
#include <QToolBar>
#include <QAction>
#include <QMenuBar>
#include <QGraphicsView>
#include <QStatusBar>
#include <QMdiArea>
#include <QDebug>
#include <QFileDialog>
#include <QMessageBox>
#include <QToolTip>
#include <QInputDialog>
#include <QGraphicsScene>
#include <QGridLayout>
#include <QFile>

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    QWidget * mainWidget;
    QGraphicsScene * scene;
    QGridLayout * mainLayout;
    QGraphicsView * view;
    QToolBar * toolBar;
    QAction *actionNewFile;
    QAction *actionOpenFile;
    QAction *actionSaveFile;
    QAction *actionSaveFileAs;
    QAction *actionExit;
    QAction *actionStart;
    QAction *actionStop;
    QAction *actionRotation;
private slots:
    void newFile();
    void openFile();
    void saveFile();
    void saveAs();
    void start();
    void stop();
    void rotation();

};
#endif // MAINWINDOW_H
