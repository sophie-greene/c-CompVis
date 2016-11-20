#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "uvccapture.h"
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv/cv.h>
using namespace cv;
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

void MainWindow::on_btnShow_clicked()
{
    Mat image=capture("/dev/video0","sna0.jpg",640,480,224,196,64,128);
    QImage img= QImage((const unsigned char*)(image.data),
    image.cols,image.rows,QImage::Format_RGB888);
    // display on label
    ui->lblCap->setPixmap(QPixmap::fromImage(img));
    // resize the label to fit the image
    ui->lblCap->resize(ui->lblCap->pixmap()->size());
}
