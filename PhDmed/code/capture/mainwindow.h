#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QtGui/QMainWindow>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv/cv.h>
using namespace cv;
namespace Ui {
    class MainWindow;
}

class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    MainWindow(QWidget *parent = 0);
    ~MainWindow();

protected:
    void changeEvent(QEvent *e);

private:
    Ui::MainWindow *ui;
    Mat imgCam0;
    Mat imgCam1;
    Mat imgCam2;
    Mat imgCam3;
    Mat imgCam4;
    Mat imgCam5;
    Mat imgCam6;
    Mat imgCam7;

    QImage qimgCam0;
    QImage qimgCam1;
    QImage qimgCam2;
    QImage qimgCam3;
    QImage qimgCam4;
    QImage qimgCam5;
    QImage qimgCam6;
    QImage qimgCam7;

    QTimer*  tmrTimer;

public slots:
    void processFrameAndUpdateGUI();
private slots:
    void on_btnShow_clicked();
};

#endif // MAINWINDOW_H
