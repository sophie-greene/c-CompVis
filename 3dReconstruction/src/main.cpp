#include <QtGui/QApplication>
#include "include/mainwindow.h"

int main(int argc, char *argv[])
{

    QApplication a(argc, argv);
    MainWindow w;
    w.show();
   // cvNamedWindow("2");
    return a.exec();
}
