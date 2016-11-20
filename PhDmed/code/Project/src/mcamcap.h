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
    CaptureThread *mThread;

private slots:
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
    void onImageAquired(bool);


};

#endif // MCAMCAP_H
