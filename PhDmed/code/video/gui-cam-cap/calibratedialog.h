#ifndef CALIBRATEDIALOG_H
#define CALIBRATEDIALOG_H
#include<QLabel>
#include <QDialog>
#include<QDesktopWidget>
#include<QtGui>
#include "calibratethread.h"

class QPushButton;
class CalibrateDialog : public QDialog {
    Q_OBJECT
public:
    CalibrateDialog(QWidget *parent = 0);
    ~CalibrateDialog();
   CalibrateThread *cThread;
   void destroyThread();
    void runCalibration();
    void calibrate();


private:
     void widgetResize();
    // QLabel* lblMsg;
    QLabel * lbl[8];
    QLabel * lblCam[8];
    QPushButton * cmdQuit;

private slots:
    void on_cmdQuit_clicked();
public slots:
    void onCalibrationSuccess(QImage,int);
    //void onCaptureComplete(vector<QImage>);
};

#endif // CALIBRATEDIALOG_H
