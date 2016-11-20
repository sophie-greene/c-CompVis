#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <jpeglib.h>
#include <time.h>
#include <linux/videodev.h>
#include<QProcess>
#include<QString>
MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::changeEvent(QEvent *e)
{
    QMainWindow::changeEvent(e);
    switch (e->type()) {
    case QEvent::LanguageChange:
        ui->retranslateUi(this);
        break;
    default:
        break;
    }
}

void MainWindow::on_btnCapture_clicked()
{
        QObject *parent;
       QString program = "./uvccapture";
       QStringList arguments;
       arguments << "-d" << "'/dev/video7'";
      // arguments << "-o" << "'x.jpg'";

       QProcess *myProcess = new QProcess(parent);
       myProcess->start(program, arguments);

    Mat frame;
   // frame=cv::imread("./x.jpg");

    //  imshow("corners",frame);



}
