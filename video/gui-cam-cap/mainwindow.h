#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include<QDialog>
#include<QDesktopWidget>
#include<QtGui>
#include<QApplication>
#include "capturethread.h"
#include "calibratethread.h"
#include<QLabel>
#define FR_WIDTH 200
#define FR_HEIGHT 150
class QPushButton;
class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    MainWindow(QWidget *parent = 0);
    ~MainWindow();
    void runCameras();
    void stopCameras();


public slots:
    void displayImage(QImage, int );
    void onCalibrationSuccess(QImage,int);
    void runCalibrate();
protected:
//void closeEvent(QCloseEvent *event);
private:
    CalibrateThread *mThread;
    CaptureThread *cThread[8];
    volatile bool areCamerasRunning;
    volatile bool isCalibrating;
    void widgetResize();
    QLabel* lblCam[8];
    QLabel*lbl[8];
    QPushButton * cmdQuit;
    QPushButton * cmdStartCameras;
    QPushButton * cmdCalibrate;

private slots:
    void on_cmdCapture_clicked();
    void on_cmdQuit_clicked();
};

#endif // MAINWINDOW_H
