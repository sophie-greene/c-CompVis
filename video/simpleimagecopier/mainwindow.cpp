#include "mainwindow.h"
#include "ui_mainwindow.h"

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

void MainWindow::on_pushButton_clicked()
{
    //serial object for /dev/ttyACM0
    QextSerialPort * port = new QextSerialPort(QLatin1String("/dev/ttyACM0"), QextSerialPort::Polling);
    VideoCapture cap;

    //write data to port
    std::string s;

    for(int cam=0;cam<8;cam++){
        cap.open(cam);
        cap.set(CV_CAP_PROP_FRAME_WIDTH, 640);
        cap.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
        cap.set(CV_CAP_PROP_FOURCC ,CV_FOURCC('Y', 'U', 'Y', 'V') );
        cap.set(CV_CAP_PROP_FPS,10);
        if(cap.isOpened()){
            for (int light=0;light<8;light++){


               s= boost::lexical_cast<std::string>(light+2);
                ///open port in write mode
               port->open(QIODevice::ReadWrite | QIODevice::Unbuffered);
               qDebug()<<s.c_str();
                port->write("1");
               port->write(s.c_str());
              Mat image;

                for (int i=0;i<40;i++) cap>>image;

                sleep(1);
                port->write("0");
                port->write(s.c_str());

               port->close();
               // namedWindow("display",1);
               // imshow("Display",image);

                string ss="cam"+boost::lexical_cast<std::string>(cam)+"light"+boost::lexical_cast<std::string>(light)+".bmp";
                imwrite(ss.c_str(),image);
            }
        }

    }
}
void MainWindow::on_actionCalibrate_triggered()
{
    ui->pushButton->setText("hello");
}
