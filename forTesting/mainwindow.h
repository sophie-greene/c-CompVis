#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include<core/core.hpp>
#include<highgui/highgui.hpp>
#include<cv.h>
#include <opencv.hpp>
#include<features2d/features2d.hpp>
#include<nonfree/features2d.hpp>
#include<calib3d/calib3d.hpp>
#include <QDebug>
#include <boost/lexical_cast.hpp>
#include<QtCore>
using namespace cv;
using namespace std;
namespace Ui {
    class MainWindow;
}

class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    MainWindow(QWidget *parent = 0);
    ~MainWindow();
    void readMat(const string& , Mat & ,Mat & ,const string & ,const string &str2);
  void detect(const string &,const string &);
  Mat remap(const Mat &, const Mat &,const Mat &);
  void Undistort(vector <string>&,const Mat &,const Mat & );
protected:
    void changeEvent(QEvent *e);

private:
    Ui::MainWindow *ui;

private slots:
    void on_pushButton_clicked();
};

#endif // MAINWINDOW_H
