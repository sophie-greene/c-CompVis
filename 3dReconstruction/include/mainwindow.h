#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include<QDialog>
#include<QDesktopWidget>
#include<QtGui>
#include<QApplication>
#include "capturethread.h"
#include "calibratethread.h"
#include "pmcalibrationthread.h"
#include<QLabel>
#include "qextserialport.h"
#include "intrcalibthread.h"
#include "laserdatathread.h"
#define FR_WIDTH 600//544x288
#define FR_HEIGHT 500
#define SERIAL "/dev/ttyACM0"
class QPushButton;
class QComboBox;
class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    MainWindow(QWidget *parent = 0);
    ~MainWindow();
    void runCameras();
    void stopCameras();


public slots:
    void displayImage(QImage, int,int,int);
    void onlaserAquired(int,int,int,int);
    void onCalibrationSuccess(QImage,int);
    void onIntrCalibrationSuccess(QImage,int);
    void onIntrCalibStopped(int,bool);
    void onpmCalibrationSuccess(bool);
    void onLaserAquis(QImage,int);
    void runCalibrate();
    void writePoint(const vector<Point> &);
    bool allFrames(const vector<Point> &);

protected:
 //   void closeEvent(QCloseEvent *event);
  //  void contextMenuEvent(QContextMenuEvent *event);
private:
    CalibrateThread *mThread;
    PhotoMCalibrationThread *pmThread;
    CaptureThread *cThread[8];
    IntrCalibThread *iThread[8];
    LaserDataThread *lThread;
    volatile bool areCamerasRunning;
    volatile bool isCalibrating;
    volatile bool isLaser;
    void widgetResize();
    void lightControl(int,bool);
    QLabel* lblCam[8];
    QLabel*lbl[8];
    QPushButton * cmdLightOn;
    QPushButton * cmdLightOff;
    QComboBox *cmbLights;
    QLineEdit *txtLightDelay;

   // QMenuBar *filemenuBar;
    QMenu * menu;
    QAction * startCamAction;
    QAction * gCalibrateAction;
    QAction * pmCalibrateAction;
    QAction * intrCalibrateAction;
    QAction * getLaser;
    QAction * quitAction;
    void creatMenu();
    vector<vector<Point> > points;
private slots:

    void on_cmdLightOn_clicked();
    void on_cmdLightOff_clicked();
    void on_menuStartCam();
    void on_menuCalib();
    void on_menuIntrCalib();
    void on_menuPMCalib();
    void on_menuLaser();
    void on_menuQuit();

};

#endif // MAINWINDOW_H
