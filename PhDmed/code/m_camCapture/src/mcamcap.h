#ifndef MCAMCAP_H
#define MCAMCAP_H

#include <QMainWindow>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv/cv.h>
#include<QtCore>
#include<QtGui>
#include<QWidget>
#include<QDebug>
#include "capturethread.h"
using namespace cv;
namespace Ui {
    class mCamCap;
}

class mCamCap : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit mCamCap(QWidget *parent = 0);
    ~mCamCap();
    CaptureThread *mThread0;
    CaptureThread *mThread1;
    CaptureThread *mThread2;
    CaptureThread *mThread3;
    CaptureThread *mThread4;
    CaptureThread *mThread5;
    CaptureThread *mThread6;
    CaptureThread *mThread7;
private slots:
    void on_cmdCalibrate_clicked();
    void on_cmdCapture_clicked();
    void calibrate();
private:
    Ui::mCamCap *ui;
    QTimer*  tmrTimer;
    QMenu *calibrateMenu;
    void createMenus();
    QAction *calibrateAction;

public slots:
    // void updateGUI();
    void onImageAquired0(bool);
    void onImageAquired1(bool);
    void onImageAquired2(bool);
    void onImageAquired3(bool);
    void onImageAquired4(bool);
    void onImageAquired5(bool);
    void onImageAquired6(bool);
    void onImageAquired7(bool);

};

#endif // MCAMCAP_H
