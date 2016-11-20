#include "mcamcap.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    mCamCap w;
    w.show();
    
    return a.exec();
}
